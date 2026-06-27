`timescale 1ns/1ps
//==========================================================================
//  tb_lz77_classic.v -- round-trip self-checking TB for the classic-LZ77
//  RTL codec. compress -> capture -> decompress -> compare to original.
//  Includes the overlap (length>offset) stress cases.
//==========================================================================
module tb_lz77_classic;
    reg clk = 0; always #5 clk = ~clk;
    reg rst_n = 0;

    // compressor (in 4096, out 16384)
    reg         c_we;  reg [11:0] c_iaddr; reg [7:0] c_idin; reg [31:0] c_ilen;
    reg         c_start; wire c_busy, c_done; wire [31:0] c_olen;
    reg [13:0]  c_oaddr; wire [7:0] c_odout;

    lz77c_compress #(.MEM_DEPTH(4096), .OUT_DEPTH(16384)) C (
        .clk(clk), .rst_n(rst_n),
        .i_we(c_we), .i_addr(c_iaddr), .i_din(c_idin), .i_len(c_ilen),
        .start(c_start), .busy(c_busy), .done(c_done), .o_len(c_olen),
        .o_addr(c_oaddr), .o_dout(c_odout));

    // decompressor (in 16384, out 4096)
    reg         d_we;  reg [13:0] d_iaddr; reg [7:0] d_idin; reg [31:0] d_ilen;
    reg         d_start; wire d_busy, d_done; wire [31:0] d_olen;
    reg [11:0]  d_oaddr; wire [7:0] d_odout;

    lz77c_decompress #(.MEM_DEPTH(16384), .OUT_DEPTH(4096)) D (
        .clk(clk), .rst_n(rst_n),
        .i_we(d_we), .i_addr(d_iaddr), .i_din(d_idin), .i_len(d_ilen),
        .start(d_start), .busy(d_busy), .done(d_done), .o_len(d_olen),
        .o_addr(d_oaddr), .o_dout(d_odout));

    reg [7:0] src  [0:4095];
    reg [7:0] comp [0:16383];
    reg [7:0] dec  [0:4095];
    integer errors = 0;

    task run_test(input [127:0] name, input integer n);
        integer i, clen, dlen, le;
        begin
            le = 0;
            @(negedge clk);
            for (i=0;i<n;i=i+1) begin c_we<=1; c_iaddr<=i[11:0]; c_idin<=src[i]; @(negedge clk); end
            c_we<=0; c_ilen<=n;
            c_start<=1; @(negedge clk); c_start<=0; wait(c_done); @(negedge clk);
            clen = c_olen;
            for (i=0;i<clen;i=i+1) begin c_oaddr<=i[13:0]; @(negedge clk); comp[i]=c_odout; end
            for (i=0;i<clen;i=i+1) begin d_we<=1; d_iaddr<=i[13:0]; d_idin<=comp[i]; @(negedge clk); end
            d_we<=0; d_ilen<=clen;
            d_start<=1; @(negedge clk); d_start<=0; wait(d_done); @(negedge clk);
            dlen = d_olen;
            if (dlen!==n) begin $display("  [%0s] LEN mismatch dec=%0d exp=%0d",name,dlen,n); le=le+1; end
            for (i=0;i<n;i=i+1) begin
                d_oaddr<=i[11:0]; @(negedge clk); dec[i]=d_odout;
                if (dec[i]!==src[i]) begin
                    if (le<4) $display("  [%0s] byte %0d got %02x exp %02x",name,i,dec[i],src[i]);
                    le=le+1;
                end
            end
            errors = errors + le;
            $display("  [%0s] in=%0d comp=%0d  %s", name, n, clen, (le==0)?"ROUND-TRIP OK":"FAIL");
        end
    endtask

    integer j;
    initial begin
        c_we=0;c_start=0;c_oaddr=0;c_iaddr=0;c_idin=0;c_ilen=0;
        d_we=0;d_start=0;d_oaddr=0;d_iaddr=0;d_idin=0;d_ilen=0;
        repeat(3) @(negedge clk); rst_n=1; repeat(2) @(negedge clk);

        // abracadabra
        begin reg [7:0] s[0:10]; integer m;
            s[0]="a";s[1]="b";s[2]="r";s[3]="a";s[4]="c";s[5]="a";s[6]="d";s[7]="a";s[8]="b";s[9]="r";s[10]="a";
            for (m=0;m<11;m=m+1) src[m]=s[m]; run_test("abracadabra",11);
        end

        // xyxyzyxyxyz
        begin reg [7:0] s[0:10]; integer m;
            s[0]="x";s[1]="y";s[2]="x";s[3]="y";s[4]="z";s[5]="y";s[6]="x";s[7]="y";s[8]="x";s[9]="y";s[10]="z";
            for (m=0;m<11;m=m+1) src[m]=s[m]; run_test("xyxyzyxyxyz",11);
        end

        // pure RLE overlap: offset=1, length>>1
        for (j=0;j<300;j=j+1) src[j]="A"; run_test("rle_A_300",300);

        // partial overlap: period-3 pattern (length can exceed offset 3)
        src[0]="X";src[1]="Y";src[2]="Z"; for (j=3;j<400;j=j+1) src[j]=src[j-3];
        run_test("xyz_period_400",400);

        // ababab... (offset 2, long overlap)
        for (j=0;j<200;j=j+1) src[j]=(j[0])?"b":"a"; run_test("abab_200",200);

        // text with repeats, longer than WINDOW (255) to exercise capped offset
        begin
            reg [7:0] p[0:11]; integer m;
            p[0]="x";p[1]="y";p[2]="x";p[3]="y";p[4]="z";p[5]="y";p[6]="x";p[7]="y";p[8]="x";p[9]="y";p[10]="z";p[11]="_";
            for (j=0;j<1500;j=j+1) src[j]=p[j%12];
            run_test("long_1500",1500);
        end

        // incompressible-ish
        for (j=0;j<400;j=j+1) src[j]=$random; run_test("random_400",400);

        // edges
        run_test("empty_0",0);
        src[0]="Q"; run_test("single_1",1);

        $display("--------------------------------------------------");
        if (errors==0) $display("LZ77_CLASSIC RTL: ALL ROUND-TRIPS PASS");
        else           $display("LZ77_CLASSIC RTL: FAIL (%0d errors)", errors);
        $finish;
    end
    initial begin #50000000; $display("TIMEOUT"); $finish; end
endmodule
