`timescale 1ns/1ps

module arinc429_system_tb;

    reg clk;
    reg reset_n;

    reg tx_start;
    reg rate_select;

    reg [7:0] label;
    reg [1:0] sdi;
    reg [18:0] data;
    reg [1:0] ssm;

    wire [1:0] tx_serial;
    wire [1:0] rx_serial;

    wire tx_busy;
    wire tx_done;

    wire [31:0] rx_word;
    wire rx_valid;
    wire rx_error;
    wire rx_busy;

    wire [7:0] decoded_label;
    wire [1:0] decoded_sdi;
    wire [18:0] decoded_data;
    wire [1:0] decoded_ssm;

    wire parity_ok;
    wire decode_valid;
    wire decode_error;

    integer errors;

    // Fault injection: invert the pulse for ARINC bit 32 (parity).
    reg inject_enable;
    reg inject_this_pulse;
    reg monitor_reset;
    reg [5:0] inject_bit_index;
    integer pulse_index;
    reg [1:0] previous_tx;

    // 1 MHz reference clock.
    initial begin
        clk = 1'b0;
        forever #500 clk = ~clk;
    end

    // Invert the selected active pulse; preserve null intervals.
    assign rx_serial =
        (inject_this_pulse && tx_serial == 2'b01) ? 2'b10 :
        (inject_this_pulse && tx_serial == 2'b10) ? 2'b01 :
        tx_serial;

    // Detect each active pulse on the falling edge, before the next
    // receiver sampling edge. pulse_index is zero-based.
    always @(negedge clk or posedge monitor_reset or negedge reset_n) begin
        if (!reset_n || monitor_reset) begin
            previous_tx      <= 2'b00;
            pulse_index      <= 0;
            inject_this_pulse <= 1'b0;
        end else begin
            if (tx_serial == 2'b00) begin
                inject_this_pulse <= 1'b0;
            end else if (previous_tx == 2'b00) begin
                if (inject_enable && pulse_index == inject_bit_index) begin
                    inject_this_pulse <= 1'b1;
                    $display("FAULT INJECTED AT ARINC BIT %0d",
                             pulse_index + 1);
                end
                pulse_index <= pulse_index + 1;
            end
            previous_tx <= tx_serial;
        end
    end

    arinc429_top DUT (
        .clk(clk),
        .reset_n(reset_n),
        .tx_start(tx_start),
        .rate_select(rate_select),
        .label(label),
        .sdi(sdi),
        .data(data),
        .ssm(ssm),
        .tx_serial(tx_serial),
        .rx_serial(rx_serial),
        .tx_busy(tx_busy),
        .tx_done(tx_done),
        .rx_word(rx_word),
        .rx_valid(rx_valid),
        .rx_error(rx_error),
        .rx_busy(rx_busy),
        .decoded_label(decoded_label),
        .decoded_sdi(decoded_sdi),
        .decoded_data(decoded_data),
        .decoded_ssm(decoded_ssm),
        .parity_ok(parity_ok),
        .decode_valid(decode_valid),
        .decode_error(decode_error)
    );

    task run_test;
        input [7:0] test_label;
        input [1:0] test_sdi;
        input [18:0] test_data;
        input [1:0] test_ssm;
        input test_rate;
        input test_inject;

        begin
            $display("");
            $display("====================================");
            $display("STARTING ARINC 429 SYSTEM TEST");
            $display("====================================");

            label      = test_label;
            sdi        = test_sdi;
            data       = test_data;
            ssm        = test_ssm;
            rate_select = test_rate;

            // Reset only the testbench fault monitor between transactions.
            monitor_reset = 1'b1;
            #1;
            monitor_reset = 1'b0;

            inject_enable    = test_inject;
            inject_bit_index = 6'd31;

            #20;

            $display("Label : %h", label);
            $display("SDI   : %b", sdi);
            $display("Data  : %h", data);
            $display("SSM   : %b", ssm);
            $display("Rate  : %s",
                     test_rate ? "100 kbps" : "12.5 kbps");

            @(negedge clk);
            tx_start = 1'b1;

            @(negedge clk);
            tx_start = 1'b0;

            wait (rx_valid == 1'b1);
            #1;

            $display("");
            $display("TRANSMISSION COMPLETE");
            $display("Received Word : %h", rx_word);
            $display("Decoded Label : %h", decoded_label);
            $display("Decoded SDI   : %b", decoded_sdi);
            $display("Decoded Data  : %h", decoded_data);
            $display("Decoded SSM   : %b", decoded_ssm);
            $display("Parity OK     : %b", parity_ok);

            if (!test_inject) begin
                if (decoded_label !== test_label ||
                    decoded_sdi !== test_sdi ||
                    decoded_data !== test_data ||
                    decoded_ssm !== test_ssm ||
                    decode_valid !== 1'b1 ||
                    decode_error !== 1'b0 ||
                    rx_error !== 1'b0) begin
                    $display("FAIL: End-to-end verification failed");
                    errors = errors + 1;
                end else begin
                    $display("PASS: End-to-end communication successful");
                    $display("PASS: Parity verified");
                end
            end else begin
                if (decode_error === 1'b1 &&
                    decode_valid === 1'b0 &&
                    parity_ok === 1'b0) begin
                    $display("PASS: Parity error detected");
                    $display("PASS: Corrupted word rejected");
                end else begin
                    $display("FAIL: Parity error not detected");
                    errors = errors + 1;
                end
            end

            wait (tx_done == 1'b1);
            @(negedge clk);

            inject_enable = 1'b0;
            $display("Transmission cycle finished.");
        end
    endtask

    initial begin
        reset_n = 1'b0;
        tx_start = 1'b0;
        rate_select = 1'b0;

        label = 8'h00;
        sdi = 2'b00;
        data = 19'h00000;
        ssm = 2'b00;

        errors = 0;

        inject_enable = 1'b0;
        inject_this_pulse = 1'b0;
        monitor_reset = 1'b0;
        inject_bit_index = 0;
        pulse_index = 0;
        previous_tx = 2'b00;

        $dumpfile("arinc429_system.vcd");
        $dumpvars(0, arinc429_system_tb);

        $display("ARINC 429 COMPLETE SYSTEM VERIFICATION");

        repeat (5) @(negedge clk);
        reset_n = 1'b1;

        // Test 1: high-speed, valid word.
        run_test(8'h83, 2'b01, 19'h12345, 2'b11, 1'b1, 1'b0);

        // Test 2: low-speed, valid word.
        run_test(8'hA5, 2'b10, 19'h45678, 2'b00, 1'b0, 1'b0);

        // Test 3: invert the transmitted parity-bit pulse.
        run_test(8'h42, 2'b11, 19'h23456, 2'b11, 1'b1, 1'b1);

        $display("");
        $display("====================================");
        if (errors == 0)
            $display("ALL SYSTEM TESTS PASSED!");
        else
            $display("SYSTEM TESTS FAILED: %0d errors", errors);
        $display("====================================");

        #1000;
        $finish;
    end

endmodule
