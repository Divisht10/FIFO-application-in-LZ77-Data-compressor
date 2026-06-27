`timescale 1ns/1ps
//==========================================================================
//  tb_lz77_stream.v
//  Streams bytes through:  compressor -> stream_fifo -> decompressor
//  with random backpressure on the input (s) and output (m) handshakes,
//  and verifies the bytes that come out equal the bytes that went in.
//==========================================================================
module tb_lz77_stream;
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n;

    // input to compressor (driven by TB)
    reg         cs_valid; wire cs_ready; reg [7:0] cs_data; reg cs_last;
    // compressor -> fifo
    wire cm_valid, cm_ready; wire [7:0] cm_data; wire cm_last;
    // fifo -> decompressor
    wire fm_valid, fm_ready; wire [7:0] fm_data; wire fm_last;
    // decompressor output (read by TB)
    wire dm_valid; reg dm_ready; wire [7:0] dm_data; wire dm_last;

    lz77_stream_compress #(.WINDOW(256), .LOOKAHEAD(16), .MAX_MATCH(15), .MAX_OFFSET(255)) C (
        .clk(clk), .rst_n(rst_n),
        .s_valid(cs_valid), .s_ready(cs_ready), .s_data(cs_data), .s_last(cs_last),
        .m_valid(cm_valid), .m_ready(cm_ready), .m_data(cm_data), .m_last(cm_last));

    stream_fifo #(.DW(8), .DEPTH(16)) F (
        .clk(clk), .rst_n(rst_n),
        .s_valid(cm_valid), .s_ready(cm_ready), .s_data(cm_data), .s_last(cm_last),
        .m_valid(fm_valid), .m_ready(fm_ready), .m_data(fm_data), .m_last(fm_last));

    lz77_stream_decompress #(.WINDOW(256)) D (
        .clk(clk), .rst_n(rst_n),
        .s_valid(fm_valid), .s_ready(fm_ready), .s_data(fm_data), .s_last(fm_last),
        .m_valid(dm_valid), .m_ready(dm_ready), .m_data(dm_data), .m_last(dm_last));

    reg [7:0] src [0:4095];
    reg [7:0] dec [0:4095];
    integer   rcount, errors;
    reg       done_o;

    // ---- input driver: one byte per call, valid/ready honored ----
    task send_byte(input [7:0] b, input lst);
        begin
            @(negedge clk); cs_valid = 1; cs_data = b; cs_last = lst;
            @(posedge clk);
            while (!cs_ready) @(posedge clk);     // hold until accepted
            @(negedge clk); cs_valid = 0; cs_last = 0;
            if ($random % 3 == 0) @(negedge clk); // random input gap
        end
    endtask

    task drive_input(input integer n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) send_byte(src[i], (i == n-1));
        end
    endtask

    // ---- output sink: random m_ready, collect until m_last ----
    task collect_output;
        begin
            rcount = 0; done_o = 0;
            while (!done_o) begin
                @(negedge clk); dm_ready = ($random % 2);
                @(posedge clk);
                if (dm_valid && dm_ready) begin
                    dec[rcount] = dm_data; rcount = rcount + 1;
                    if (dm_last) done_o = 1;
                end
            end
            @(negedge clk); dm_ready = 0;
        end
    endtask

    task run_test(input [127:0] name, input integer n);
        integer i, le;
        begin
            le = 0;
            rst_n = 0; cs_valid = 0; cs_last = 0; dm_ready = 0;
            repeat (3) @(negedge clk); rst_n = 1; @(negedge clk);
            fork
                drive_input(n);
                collect_output;
            join
            if (rcount !== n) begin
                $display("  [%0s] LEN mismatch out=%0d exp=%0d", name, rcount, n); le = le + 1;
            end
            for (i = 0; i < n; i = i + 1)
                if (dec[i] !== src[i]) begin
                    if (le < 4) $display("  [%0s] byte %0d got %02x exp %02x", name, i, dec[i], src[i]);
                    le = le + 1;
                end
            errors = errors + le;
            $display("  [%0s] in=%0d out=%0d  %s", name, n, rcount, (le==0)?"ROUND-TRIP OK":"FAIL");
        end
    endtask

    integer j;
    initial begin
        errors = 0; cs_valid = 0; dm_ready = 0; rst_n = 0;

        begin : tc_abra
            reg [7:0] s[0:10]; integer m;
            s[0]="a";s[1]="b";s[2]="r";s[3]="a";s[4]="c";s[5]="a";s[6]="d";s[7]="a";s[8]="b";s[9]="r";s[10]="a";
            for (m=0;m<11;m=m+1) src[m]=s[m]; run_test("abracadabra",11);
        end
        begin : tc_xyz
            reg [7:0] s[0:10]; integer m;
            s[0]="x";s[1]="y";s[2]="x";s[3]="y";s[4]="z";s[5]="y";s[6]="x";s[7]="y";s[8]="x";s[9]="y";s[10]="z";
            for (m=0;m<11;m=m+1) src[m]=s[m]; run_test("xyxyzyxyxyz",11);
        end

        for (j=0;j<300;j=j+1) src[j]="A";              run_test("rle_A_300",300);
        src[0]="X";src[1]="Y";src[2]="Z";
        for (j=3;j<400;j=j+1) src[j]=src[j-3];          run_test("xyz_period_400",400);
        for (j=0;j<200;j=j+1) src[j]=(j[0])?"b":"a";    run_test("abab_200",200);

        begin : tc_long
            reg [7:0] p[0:11]; integer m;
            p[0]="x";p[1]="y";p[2]="x";p[3]="y";p[4]="z";p[5]="y";p[6]="x";p[7]="y";p[8]="x";p[9]="y";p[10]="z";p[11]="_";
            for (j=0;j<1200;j=j+1) src[j]=p[j%12];      run_test("long_1200",1200);
        end

        for (j=0;j<400;j=j+1) src[j]=$random;           run_test("random_400",400);
        src[0]="Q";                                     run_test("single_1",1);

        $display("--------------------------------------------------");
        if (errors==0) $display("LZ77 STREAMING: ALL ROUND-TRIPS PASS");
        else           $display("LZ77 STREAMING: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin #200000000; $display("STREAMING: TIMEOUT (rcount=%0d)", rcount); $finish; end
endmodule
