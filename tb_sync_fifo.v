`timescale 1ns/1ps
//==========================================================================
//  tb_sync_fifo.v -- self-checking testbench for the synchronous FIFO
//==========================================================================
module tb_sync_fifo;
    localparam DW = 8, DEPTH = 8, AW = 3;

    reg            clk = 0, rst_n = 0;
    reg            wr_en = 0, rd_en = 0;
    reg  [DW-1:0]  din = 0;
    wire [DW-1:0]  dout;
    wire           full, empty;

    sync_fifo #(.DATA_WIDTH(DW), .DEPTH(DEPTH)) dut (
        .clk(clk), .rst_n(rst_n), .wr_en(wr_en), .rd_en(rd_en),
        .din(din), .dout(dout), .full(full), .empty(empty));

    always #5 clk = ~clk;     // 100 MHz

    // reference model: a queue
    reg [DW-1:0] model [0:1023];
    integer wn = 0, rn = 0, errors = 0, i;

    task do_push(input [DW-1:0] v);
        begin
            @(negedge clk);
            if (!full) begin
                din = v; wr_en = 1;
                model[wn] = v; wn = wn + 1;
            end
            @(negedge clk); wr_en = 0;
        end
    endtask

    task do_pop;
        begin
            @(negedge clk);
            if (!empty) begin
                rd_en = 1;
                @(negedge clk); rd_en = 0;
                #1;                              // let registered dout settle
                if (dout !== model[rn]) begin
                    $display("  MISMATCH at read %0d: got %02x exp %02x",
                              rn, dout, model[rn]);
                    errors = errors + 1;
                end
                rn = rn + 1;
            end else @(negedge clk);
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;

        // 1) fill to full, confirm full asserts
        for (i = 0; i < DEPTH; i = i + 1) do_push(i + 8'hA0);
        @(negedge clk);
        if (!full)  begin $display("  ERROR: full not asserted"); errors=errors+1; end

        // 2) drain completely, confirm empty asserts and data matches
        for (i = 0; i < DEPTH; i = i + 1) do_pop;
        @(negedge clk);
        if (!empty) begin $display("  ERROR: empty not asserted"); errors=errors+1; end

        // 3) interleaved traffic
        for (i = 0; i < 40; i = i + 1) begin
            do_push($random);
            if (i % 2 == 0) do_pop;
        end
        while (!empty) do_pop;

        if (errors == 0) $display("SYNC_FIFO: PASS (%0d words checked)", rn);
        else             $display("SYNC_FIFO: FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
