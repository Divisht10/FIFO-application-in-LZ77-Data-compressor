//==========================================================================
//  lz77_stream.v -- STREAMING classic-LZ77 codec (offset,length,next_char).
//
//  Dataflow, not block: bytes arrive on a valid/ready handshake, tokens
//  leave on a valid/ready handshake, end-of-stream travels as `last`.
//  No frame is buffered whole -- only a sliding window is held.
//
//  Compressor state:
//    hist[]  : circular history window (the search buffer), hwr/hcount
//    look[]  : small look-ahead shift register, lcount valid bytes
//  A token can be emitted once the look-ahead is full OR the input ended.
//  Overlap (length>offset) works because the match source is taken from
//  the look-ahead once it runs past the cursor; the decoder copies one
//  byte per cycle so it reproduces bytes it just wrote.
//==========================================================================

// -------------------------------------------------------------------------
//  Streaming compressor
// -------------------------------------------------------------------------
module lz77_stream_compress #(
    parameter integer DW         = 8,
    parameter integer WINDOW     = 256,      // history size (power of 2)
    parameter integer MAX_OFFSET = 255,      // largest back-distance (fits a byte)
    parameter integer LOOKAHEAD  = 16,       // >= MAX_MATCH+1
    parameter integer MAX_MATCH  = 15,       // <= LOOKAHEAD-1 (fits a byte)
    parameter integer HAW        = $clog2(WINDOW),
    parameter integer LAW        = $clog2(LOOKAHEAD)
)(
    input  wire           clk,
    input  wire           rst_n,
    // input byte stream
    input  wire           s_valid,
    output wire           s_ready,
    input  wire [DW-1:0]  s_data,
    input  wire           s_last,
    // output token-byte stream
    output wire           m_valid,
    input  wire           m_ready,
    output wire [DW-1:0]  m_data,
    output wire           m_last
);
    integer ii;

    reg [DW-1:0] hist [0:WINDOW-1];
    reg [HAW-1:0] hwr;
    reg [HAW:0]   hcount;

    reg [DW-1:0] look [0:LOOKAHEAD-1];
    reg [LAW:0]  lcount;
    reg          ended;

    localparam [3:0] S_FILL=0, S_SEARCH=1, S_E0=2, S_E1=3, S_E2=4, S_ADV=5, S_DONE=6;
    reg [3:0] state;

    reg [HAW:0]  d;
    reg [DW-1:0] best_off;
    reg [LAW:0]  k;
    reg [DW-1:0] best_len;
    reg [LAW:0]  adv_j;

    // longest legal match this step (leave 1 byte for next_char)
    wire [LAW:0] maxm = (lcount == 0) ? 0 :
                        ((lcount - 1) < MAX_MATCH) ? (lcount - 1'b1) : MAX_MATCH[LAW:0];
    wire [HAW:0] dlimit = (hcount < MAX_OFFSET) ? hcount : MAX_OFFSET[HAW:0];

    // match source: history for k<d, else into the look-ahead (overlap)
    wire [HAW-1:0] sidx = hwr - d[HAW-1:0] + k[HAW-1:0];
    wire [DW-1:0]  src  = (k < d) ? hist[sidx] : look[k - d];
    wire           can_extend = (k < maxm) && (src == look[k]);

    wire can_accept = (lcount < LOOKAHEAD) && ~ended;
    wire final_tok  = ended && ((best_len + 1'b1) == lcount);

    assign s_ready = (state == S_FILL) && can_accept;
    assign m_valid = (state == S_E0) || (state == S_E1) || (state == S_E2);
    assign m_data  = (state == S_E0) ? best_off :
                     (state == S_E1) ? best_len : look[best_len];
    assign m_last  = (state == S_E2) && final_tok;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_FILL; hwr <= 0; hcount <= 0; lcount <= 0; ended <= 0;
        end else begin
            case (state)
            S_FILL: begin
                if (s_valid && s_ready) begin
                    look[lcount] <= s_data; lcount <= lcount + 1'b1;
                    if (s_last) ended <= 1'b1;
                end else if (lcount == LOOKAHEAD || (ended && lcount != 0)) begin
                    best_len <= 0; best_off <= 0; d <= 1; k <= 0;
                    state <= (hcount == 0) ? S_E0 : S_SEARCH;
                end else if (ended && lcount == 0) begin
                    state <= S_DONE;
                end
            end
            S_SEARCH: begin
                if (can_extend) k <= k + 1'b1;
                else begin
                    if (k > best_len) begin best_len <= k; best_off <= d; end
                    if (d == dlimit) state <= S_E0;
                    else begin d <= d + 1'b1; k <= 0; end
                end
            end
            S_E0: if (m_ready) state <= S_E1;
            S_E1: if (m_ready) state <= S_E2;
            S_E2: if (m_ready) begin adv_j <= 0; state <= S_ADV; end
            // advance window by (best_len+1): push look[0] -> history, shift look
            S_ADV: begin
                hist[hwr] <= look[0];
                hwr <= hwr + 1'b1;
                if (hcount < WINDOW) hcount <= hcount + 1'b1;
                for (ii = 0; ii < LOOKAHEAD-1; ii = ii + 1) look[ii] <= look[ii+1];
                look[LOOKAHEAD-1] <= 0;
                lcount <= lcount - 1'b1;
                if (adv_j == best_len) state <= S_FILL;
                else adv_j <= adv_j + 1'b1;
            end
            S_DONE: state <= S_DONE;
            default: state <= S_FILL;
            endcase
        end
    end
endmodule


// -------------------------------------------------------------------------
//  Streaming decompressor
// -------------------------------------------------------------------------
module lz77_stream_decompress #(
    parameter integer DW     = 8,
    parameter integer WINDOW = 256,          // power of 2
    parameter integer HAW    = $clog2(WINDOW)
)(
    input  wire           clk,
    input  wire           rst_n,
    input  wire           s_valid,
    output wire           s_ready,
    input  wire [DW-1:0]  s_data,
    input  wire           s_last,
    output wire           m_valid,
    input  wire           m_ready,
    output wire [DW-1:0]  m_data,
    output wire           m_last
);
    reg [DW-1:0]  hist [0:WINDOW-1];
    reg [HAW-1:0] hwr;

    localparam [2:0] D_OFF=0, D_LEN=1, D_CHR=2, D_COPY=3, D_WRC=4;
    reg [2:0] state;

    reg [DW-1:0] off_r, len_r, nextc;
    reg [DW-1:0] mk;
    reg          final_r;

    wire [HAW-1:0] sidx  = hwr - off_r[HAW-1:0];
    wire [DW-1:0]  cpy   = hist[sidx];

    assign s_ready = (state == D_OFF) || (state == D_LEN) || (state == D_CHR);
    assign m_valid = (state == D_COPY) || (state == D_WRC);
    assign m_data  = (state == D_COPY) ? cpy : nextc;
    assign m_last  = (state == D_WRC) && final_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin state <= D_OFF; hwr <= 0; end
        else begin
            case (state)
            D_OFF: if (s_valid) begin off_r <= s_data; state <= D_LEN; end
            D_LEN: if (s_valid) begin len_r <= s_data; mk <= 0; state <= D_CHR; end
            D_CHR: if (s_valid) begin
                       nextc   <= s_data;
                       final_r <= s_last;
                       state   <= (len_r == 0) ? D_WRC : D_COPY;
                   end
            D_COPY: if (m_ready) begin
                       hist[hwr] <= cpy; hwr <= hwr + 1'b1;   // append copied byte
                       if (mk + 1 >= len_r) state <= D_WRC;
                       else mk <= mk + 1'b1;
                   end
            D_WRC: if (m_ready) begin
                       hist[hwr] <= nextc; hwr <= hwr + 1'b1;
                       state <= D_OFF;
                   end
            default: state <= D_OFF;
            endcase
        end
    end
endmodule
