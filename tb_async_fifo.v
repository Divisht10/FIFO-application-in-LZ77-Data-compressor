`timescale 1ns/1ps
//==========================================================================
//  tb_async_fifo.v -- self-checking testbench for the asynchronous FIFO
//  Write clock and read clock run at deliberately different rates.
//==========================================================================
module tb_async_fifo;
    localparam DW = 8, AW = 4;          // depth = 16

    reg            wclk = 0, rclk = 0;
    reg            wrst_n = 0, rrst_n = 0;
    reg            winc = 0, rinc = 0;
    reg  [DW-1:0]  wdata = 0;
    wire [DW-1:0]  rdata;
    wire           wfull, rempty;

    async_fifo #(.DATA_WIDTH(DW), .ADDR_WIDTH(AW)) dut (
        .wclk(wclk), .wrst_n(wrst_n), .rclk(rclk), .rrst_n(rrst_n),
        .winc(winc), .rinc(rinc), .wdata(wdata),
        .rdata(rdata), .wfull(wfull), .rempty(rempty));

    // asymmetric clocks: write faster than read
    always #5  wclk = ~wclk;            // 100 MHz
    always #8  rclk = ~rclk;            //  62.5 MHz

    // reference queue
    reg [DW-1:0] model [0:4095];
    integer wn = 0, rn = 0, errors = 0, i;

    localparam integer N = 200;

    // ---- writer process: keep going until N words are actually pushed ----
    initial begin
        repeat (4) @(negedge wclk);
        wrst_n = 1;
        while (wn < N) begin
            @(negedge wclk);
            if (!wfull) begin
                wdata = $random;
                winc  = 1;
                model[wn] = wdata; wn = wn + 1;
            end else winc = 0;
            @(negedge wclk); winc = 0;
            if ($random % 3 == 0) @(negedge wclk);   // random gaps
        end
        winc = 0;
    end

    // ---- reader process ----
    initial begin
        repeat (4) @(negedge rclk);
        rrst_n = 1;
        // read until we've pulled everything the writer produced
        while (rn < N) begin
            @(negedge rclk);
            if (!rempty) begin
                if (rdata !== model[rn]) begin
                    $display("  MISMATCH at read %0d: got %02x exp %02x",
                              rn, rdata, model[rn]);
                    errors = errors + 1;
                end
                rn   = rn + 1;
                rinc = 1;
            end else rinc = 0;
            @(negedge rclk); rinc = 0;
        end

        if (errors == 0) $display("ASYNC_FIFO: PASS (%0d words crossed clocks)", rn);
        else             $display("ASYNC_FIFO: FAIL (%0d errors)", errors);
        $finish;
    end

    // safety timeout
    initial begin
        #200000;
        $display("ASYNC_FIFO: TIMEOUT (wrote %0d, read %0d)", wn, rn);
        $finish;
    end
endmodule
