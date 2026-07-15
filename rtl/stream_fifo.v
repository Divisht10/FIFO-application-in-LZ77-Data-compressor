//==========================================================================
//  stream_fifo.v -- first-word-fall-through sync FIFO with a valid/ready
//  (AXI-Stream-like) interface. Carries one `last` bit alongside each byte
//  so end-of-stream propagates through the buffer.
//
//  This is the same ring-buffer + wrap-bit pointer scheme as sync_fifo.v,
//  just dressed in handshake clothing:
//     s_ready = ~full         (we can accept when not full)
//     m_valid = ~empty        (data is available when not empty)
//  A beat moves on (valid && ready). FWFT means m_data is presented
//  combinationally, no read latency -- ideal for chaining stream blocks.
//==========================================================================
module stream_fifo #(
    parameter integer DW    = 8,
    parameter integer DEPTH = 16,            // power of 2
    parameter integer AW    = $clog2(DEPTH)
)(
    input  wire           clk,
    input  wire           rst_n,
    // slave (input) side
    input  wire           s_valid,
    output wire           s_ready,
    input  wire [DW-1:0]  s_data,
    input  wire           s_last,
    // master (output) side
    output wire           m_valid,
    input  wire           m_ready,
    output wire [DW-1:0]  m_data,
    output wire           m_last
);
    reg [DW:0]   mem [0:DEPTH-1];            // {last, data}
    reg [AW:0]   wptr, rptr;

    wire full  = (wptr[AW] != rptr[AW]) && (wptr[AW-1:0] == rptr[AW-1:0]);
    wire empty = (wptr == rptr);

    assign s_ready = ~full;
    assign m_valid = ~empty;
    assign {m_last, m_data} = mem[rptr[AW-1:0]];

    wire do_wr = s_valid & s_ready;
    wire do_rd = m_valid & m_ready;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) wptr <= 0;
        else if (do_wr) begin
            mem[wptr[AW-1:0]] <= {s_last, s_data};
            wptr <= wptr + 1'b1;
        end

    always @(posedge clk or negedge rst_n)
        if (!rst_n) rptr <= 0;
        else if (do_rd) rptr <= rptr + 1'b1;
endmodule
