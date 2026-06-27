//==========================================================================
//  lz77_classic.v -- classic LZ77 (offset,length,next_char) codec.
//  Matches lz77_robust.c: every token copies `length` bytes from `offset`
//  back, then appends `next_char`. offset==0 means a pure literal.
//
//  Compressed stream: [4-byte little-endian original length], then
//  repeated 3-byte tokens { offset, length, next_char }.
//
//  Overlap (length > offset) is handled the same way as the C decoder:
//  the decoder copies ONE byte per cycle, so it can read bytes it just
//  produced. The encoder matches over the input buffer (which already
//  holds the look-ahead bytes), so it can find such matches.
//
//  Memory-mapped block interface (the async FIFOs drive these on FPGA):
//    load via (i_we,i_addr,i_din)+i_len, pulse start, wait done,
//    read result via (o_addr,o_dout), size in o_len.
//==========================================================================

// -------------------------------------------------------------------------
//  Compressor
// -------------------------------------------------------------------------
module lz77c_compress #(
    parameter integer DATA_WIDTH = 8,
    parameter integer WINDOW     = 255,    // max offset (1..WINDOW); 0 = no match
    parameter integer MAX_MATCH  = 63,     // max copy length
    parameter integer MEM_DEPTH  = 4096,
    parameter integer OUT_DEPTH  = 16384,  // worst case ~3x input
    parameter integer AW         = $clog2(MEM_DEPTH),
    parameter integer OAW        = $clog2(OUT_DEPTH),
    parameter integer LENW       = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  i_we,
    input  wire [AW-1:0]         i_addr,
    input  wire [DATA_WIDTH-1:0] i_din,
    input  wire [LENW-1:0]       i_len,
    input  wire                  start,
    output reg                   busy,
    output reg                   done,
    output reg  [LENW-1:0]       o_len,
    input  wire [OAW-1:0]        o_addr,
    output wire [DATA_WIDTH-1:0] o_dout
);
    reg [DATA_WIDTH-1:0] inbuf  [0:MEM_DEPTH-1];
    reg [DATA_WIDTH-1:0] outbuf [0:OUT_DEPTH-1];

    always @(posedge clk) if (i_we) inbuf[i_addr] <= i_din;
    assign o_dout = outbuf[o_addr];

    localparam [3:0]
        S_IDLE=0, S_H0=1, S_H1=2, S_H2=3, S_H3=4,
        S_TOK=5, S_SEARCH=6, S_E0=7, S_E1=8, S_E2=9, S_TOKDONE=10, S_DONE=11;
    reg [3:0] state;

    reg [LENW-1:0] len;
    reg [AW:0]     pos;
    reg [OAW-1:0]  outptr;
    reg [AW:0]     off, k;
    reg [AW:0]     best_len, best_off;

    // remaining = len - pos ; max match leaves 1 byte for next_char
    wire [LENW-1:0] remaining = len - pos;
    wire [AW:0] maxoff_c   = (pos < WINDOW) ? pos[AW:0] : WINDOW[AW:0];
    wire [AW:0] maxmatch_c = (remaining <= 1) ? {(AW+1){1'b0}} :
                             ((remaining - 1) < MAX_MATCH) ? (remaining[AW:0] - 1'b1)
                                                           : MAX_MATCH[AW:0];

    // dual read of inbuf for the compare
    wire [DATA_WIDTH-1:0] a_byte = inbuf[pos + k];
    wire [DATA_WIDTH-1:0] b_byte = inbuf[pos - off + k];
    wire can_extend = (k < maxmatch_c) && (a_byte == b_byte);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; busy <= 0; done <= 0; o_len <= 0;
        end else begin
            case (state)
            S_IDLE: begin
                done <= 1'b0;
                if (start) begin
                    len <= i_len; pos <= 0; outptr <= 0; busy <= 1'b1; state <= S_H0;
                end
            end
            S_H0: begin outbuf[outptr] <= len[7:0];   outptr <= outptr+1; state <= S_H1; end
            S_H1: begin outbuf[outptr] <= len[15:8];  outptr <= outptr+1; state <= S_H2; end
            S_H2: begin outbuf[outptr] <= len[23:16]; outptr <= outptr+1; state <= S_H3; end
            S_H3: begin outbuf[outptr] <= len[31:24]; outptr <= outptr+1; state <= S_TOK; end

            S_TOK: begin
                if (pos >= len) state <= S_DONE;
                else begin
                    best_len <= 0; best_off <= 0; off <= 1; k <= 0;
                    state <= (maxoff_c == 0 || maxmatch_c == 0) ? S_E0 : S_SEARCH;
                end
            end
            // longest-match search, one compare per cycle
            S_SEARCH: begin
                if (can_extend) k <= k + 1;
                else begin
                    if (k > best_len) begin best_len <= k; best_off <= off; end
                    if (off == maxoff_c) state <= S_E0;
                    else begin off <= off + 1; k <= 0; end
                end
            end
            // emit 3-byte token: offset, length, next_char
            S_E0: begin outbuf[outptr] <= best_off;            outptr <= outptr+1; state <= S_E1; end
            S_E1: begin outbuf[outptr] <= best_len;            outptr <= outptr+1; state <= S_E2; end
            S_E2: begin outbuf[outptr] <= inbuf[pos+best_len]; outptr <= outptr+1; state <= S_TOKDONE; end

            S_TOKDONE: begin pos <= pos + best_len + 1; state <= S_TOK; end
            S_DONE:    begin o_len <= outptr; busy <= 1'b0; done <= 1'b1; state <= S_IDLE; end
            default:   state <= S_IDLE;
            endcase
        end
    end
endmodule


// -------------------------------------------------------------------------
//  Decompressor -- byte-at-a-time copy makes length > offset correct.
// -------------------------------------------------------------------------
module lz77c_decompress #(
    parameter integer DATA_WIDTH = 8,
    parameter integer MEM_DEPTH  = 16384,
    parameter integer OUT_DEPTH  = 4096,
    parameter integer AW         = $clog2(MEM_DEPTH),
    parameter integer OAW        = $clog2(OUT_DEPTH),
    parameter integer LENW       = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  i_we,
    input  wire [AW-1:0]         i_addr,
    input  wire [DATA_WIDTH-1:0] i_din,
    input  wire [LENW-1:0]       i_len,
    input  wire                  start,
    output reg                   busy,
    output reg                   done,
    output reg  [LENW-1:0]       o_len,
    input  wire [OAW-1:0]        o_addr,
    output wire [DATA_WIDTH-1:0] o_dout
);
    reg [DATA_WIDTH-1:0] cbuf [0:MEM_DEPTH-1];
    reg [DATA_WIDTH-1:0] dbuf [0:OUT_DEPTH-1];

    always @(posedge clk) if (i_we) cbuf[i_addr] <= i_din;
    assign o_dout = dbuf[o_addr];

    localparam [3:0]
        S_IDLE=0, S_H0=1, S_H1=2, S_H2=3, S_H3=4,
        S_RDOFF=5, S_RDLEN=6, S_RDCHR=7, S_COPY=8, S_WRCHR=9, S_DONE=10;
    reg [3:0] state;

    reg [LENW-1:0] olen;
    reg [AW:0]     cptr;
    reg [OAW:0]    outpos;
    reg [OAW:0]    off_r, mlen, mk;
    reg [DATA_WIDTH-1:0] nextc;

    wire [DATA_WIDTH-1:0] cbyte = cbuf[cptr];
    wire [DATA_WIDTH-1:0] hbyte = dbuf[outpos - off_r];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; busy <= 0; done <= 0; o_len <= 0;
        end else begin
            case (state)
            S_IDLE: begin
                done <= 1'b0;
                if (start) begin cptr <= 0; outpos <= 0; busy <= 1'b1; state <= S_H0; end
            end
            S_H0: begin olen[7:0]   <= cbyte; cptr <= cptr+1; state <= S_H1; end
            S_H1: begin olen[15:8]  <= cbyte; cptr <= cptr+1; state <= S_H2; end
            S_H2: begin olen[23:16] <= cbyte; cptr <= cptr+1; state <= S_H3; end
            S_H3: begin olen[31:24] <= cbyte; cptr <= cptr+1; state <= S_RDOFF; end

            S_RDOFF: begin
                if (outpos >= olen) state <= S_DONE;
                else begin off_r <= cbyte; cptr <= cptr+1; state <= S_RDLEN; end
            end
            S_RDLEN: begin mlen  <= cbyte; cptr <= cptr+1; mk <= 0; state <= S_RDCHR; end
            S_RDCHR: begin
                nextc <= cbyte; cptr <= cptr+1;
                state <= (mlen == 0) ? S_WRCHR : S_COPY;   // mlen captured last cycle
            end
            S_COPY: begin
                dbuf[outpos] <= hbyte; outpos <= outpos + 1;
                if (mk + 1 >= mlen) state <= S_WRCHR;
                else                mk <= mk + 1;
            end
            S_WRCHR: begin
                dbuf[outpos] <= nextc; outpos <= outpos + 1; state <= S_RDOFF;
            end
            S_DONE:  begin o_len <= outpos; busy <= 1'b0; done <= 1'b1; state <= S_IDLE; end
            default: state <= S_IDLE;
            endcase
        end
    end
endmodule
