`timescale 1ns/1ps
//==========================================================================
//  tb_revised_async_fifo.v
//  Self-checking testbench for the revised (non-power-of-two) async FIFO.
//
//  What it verifies:
//   (1) CAPACITY  -- from empty, with no reads, the FIFO accepts exactly
//                    DEPTH words before wfull asserts. This is the headline
//                    claim of the revision (true DEPTH, not a power of two).
//   (2) INTEGRITY -- N words streamed across two asymmetric clocks come out
//                    in order, byte-for-byte, checked against a reference
//                    queue. N >> DEPTH and >> 2^CDC_WIDTH so both the
//                    modulo-DEPTH address counter and the power-of-2 CDC
//                    counter wrap many times.
//==========================================================================
module tb_revised_async_fifo;
    localparam integer DW    = 8;
    localparam integer DEPTH = 12;     // non-power-of-two on purpose
    localparam integer N     = 200;    // total words to stream

    reg            wclk = 0, rclk = 0;
    reg            wrst_n = 0, rrst_n = 0;
    reg            winc = 0, rinc = 0;
    reg  [DW-1:0]  wdata = 0;
    wire [DW-1:0]  rdata;
    wire           wfull, rempty;

    async_fifo #(.DATA_WIDTH(DW), .DEPTH(DEPTH)) dut (
        .wclk(wclk), .wrst_n(wrst_n), .rclk(rclk), .rrst_n(rrst_n),
        .winc(winc), .rinc(rinc), .wdata(wdata),
        .rdata(rdata), .wfull(wfull), .rempty(rempty));

    // asymmetric clocks: writer faster than reader
    always #5 wclk = ~wclk;            // 100 MHz
    always #8 rclk = ~rclk;            //  62.5 MHz

    // reference queue (linear history of everything written, in order)
    reg [DW-1:0] model [0:4095];
    integer wn = 0, rn = 0, errors = 0;
    integer cap_count;
    reg     cap_done = 0;

    // ---------------- reset ----------------
    initial begin
        wrst_n = 0; rrst_n = 0;
        repeat (4) @(negedge wclk);
        @(negedge rclk);
        wrst_n = 1;
        rrst_n = 1;
    end

    // ---------------- capacity check (writer owns this phase) ----------------
    task capacity_check;
        begin
            cap_count = 0;
            winc      = 0;
            while (!wfull) begin
                @(negedge wclk);
                if (!wfull) begin                 // re-check: wfull is registered
                    wdata      = 8'hC0 + cap_count;
                    winc       = 1;
                    model[wn]  = wdata; wn = wn + 1;
                    cap_count  = cap_count + 1;
                end
                @(negedge wclk); winc = 0;
            end
            winc = 0;
            if (cap_count !== DEPTH) begin
                $display("  CAPACITY FAIL: accepted %0d, expected %0d", cap_count, DEPTH);
                errors = errors + 1;
            end else
                $display("  capacity OK: holds exactly %0d words", cap_count);
        end
    endtask

    // ---------------- writer ----------------
    initial begin
        winc = 0;
        wait (wrst_n);
        @(negedge wclk);
        capacity_check;                           // fill to DEPTH, no reads yet
        cap_done = 1;
        while (wn < N) begin                      // stream the rest
            @(negedge wclk);
            if (!wfull) begin
                wdata     = $random;
                winc      = 1;
                model[wn] = wdata; wn = wn + 1;
            end else winc = 0;
            @(negedge wclk); winc = 0;
            if ($random % 3 == 0) @(negedge wclk); // random gaps
        end
        winc = 0;
    end

    // ---------------- reader ----------------
    initial begin
        rinc = 0;
        wait (rrst_n);
        wait (cap_done);                          // measure capacity before draining
        while (rn < N) begin
            @(negedge rclk);
            if (!rempty) begin
                if (rdata !== model[rn]) begin
                    $display("  MISMATCH read %0d: got %02x exp %02x",
                              rn, rdata, model[rn]);
                    errors = errors + 1;
                end
                rn   = rn + 1;
                rinc = 1;
            end else rinc = 0;
            @(negedge rclk); rinc = 0;
        end

        if (errors == 0)
            $display("REVISED_ASYNC_FIFO: PASS (%0d words crossed, DEPTH=%0d)", rn, DEPTH);
        else
            $display("REVISED_ASYNC_FIFO: FAIL (%0d errors)", errors);
        $finish;
    end

    // ---------------- safety timeout ----------------
    initial begin
        #500000;
        $display("REVISED_ASYNC_FIFO: TIMEOUT (wn=%0d rn=%0d, errors=%0d)", wn, rn, errors);
        $finish;
    end
endmodule
