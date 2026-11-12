module arinc429_top (
    input wire clk,
    input wire reset_n,

    input wire tx_start,
    input wire rate_select,

    input wire [7:0] label,
    input wire [1:0] sdi,
    input wire [18:0] data,
    input wire [1:0] ssm,

    output wire [1:0] tx_serial,
    input wire [1:0] rx_serial,

    output wire tx_busy,
    output wire tx_done,

    output wire [31:0] rx_word,
    output wire rx_valid,
    output wire rx_error,
    output wire rx_busy,

    output wire [7:0] decoded_label,
    output wire [1:0] decoded_sdi,
    output wire [18:0] decoded_data,
    output wire [1:0] decoded_ssm,

    output wire parity_ok,
    output wire decode_valid,
    output wire decode_error
);

    wire [31:0] encoded_word;
    wire encoded_parity;

    arinc429_encoder ENCODER (
        .label(label),
        .sdi(sdi),
        .data(data),
        .ssm(ssm),
        .arinc_word(encoded_word),
        .parity(encoded_parity)
    );

    arinc429_transmitter TRANSMITTER (
        .clk(clk),
        .reset_n(reset_n),
        .tx_start(tx_start),
        .rate_select(rate_select),
        .data_word(encoded_word),
        .tx_out(tx_serial),
        .tx_busy(tx_busy),
        .tx_done(tx_done)
    );

    arinc429_receiver RECEIVER (
        .clk(clk),
        .reset_n(reset_n),
        .rx_in(rx_serial),
        .rate_select(rate_select),
        .rx_word(rx_word),
        .rx_valid(rx_valid),
        .rx_error(rx_error),
        .rx_busy(rx_busy)
    );

    arinc429_decoder DECODER (
        .rx_word(rx_word),
        .rx_valid(rx_valid),
        .decoded_label(decoded_label),
        .decoded_sdi(decoded_sdi),
        .decoded_data(decoded_data),
        .decoded_ssm(decoded_ssm),
        .parity_ok(parity_ok),
        .decode_valid(decode_valid),
        .decode_error(decode_error)
    );

endmodule


module arinc429_receiver (
    input  wire        clk,
    input  wire        reset_n,
    input  wire [1:0]  rx_in,       // 01 = positive, 10 = negative, 00 = null
    input  wire        rate_select, // 1 = 100 kbps, 0 = 12.5 kbps
    output reg  [31:0] rx_word,
    output reg         rx_valid,
    output reg         rx_error,
    output reg         rx_busy
);

    localparam IDLE    = 1'b0;
    localparam RECEIVE = 1'b1;

    reg state;
    reg [6:0] bit_timer;
    reg [6:0] bit_period;
    reg [5:0] bit_index;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state      <= IDLE;
            rx_word    <= 32'b0;
            rx_valid   <= 1'b0;
            rx_error   <= 1'b0;
            rx_busy    <= 1'b0;
            bit_timer  <= 0;
            bit_period <= 0;
            bit_index  <= 0;
        end else begin
            rx_valid <= 1'b0;
            rx_error <= 1'b0;

            case (state)
                IDLE: begin
                    rx_busy <= 1'b0;

                    if (rx_in == 2'b01 || rx_in == 2'b10) begin
                        bit_period <= rate_select ? 7'd10 : 7'd80;
                        bit_timer  <= 0;
                        rx_word[0] <= (rx_in == 2'b01);
                        bit_index  <= 1;
                        rx_busy    <= 1'b1;
                        state      <= RECEIVE;
                    end else if (rx_in == 2'b11) begin
                        rx_error <= 1'b1;
                    end
                end

                RECEIVE: begin
                    if (bit_timer == bit_period - 1'b1) begin
                        bit_timer <= 0;

                        if (rx_in == 2'b01 || rx_in == 2'b10) begin
                            rx_word[bit_index] <= (rx_in == 2'b01);

                            if (bit_index == 6'd31) begin
                                rx_valid <= 1'b1;
                                rx_busy  <= 1'b0;
                                state    <= IDLE;
                            end else begin
                                bit_index <= bit_index + 1'b1;
                            end
                        end else begin
                            rx_error <= 1'b1;
                            rx_busy  <= 1'b0;
                            state    <= IDLE;
                        end
                    end else begin
                        bit_timer <= bit_timer + 1'b1;
                    end
                end

                default: begin
                    state   <= IDLE;
                    rx_busy <= 1'b0;
                end
            endcase
        end
    end

endmodule


module arinc429_decoder (
    input  wire [31:0] rx_word,
    input  wire        rx_valid,
    output reg  [7:0]  decoded_label,
    output reg  [1:0]  decoded_sdi,
    output reg  [18:0] decoded_data,
    output reg  [1:0]  decoded_ssm,
    output reg         parity_ok,
    output wire        decode_valid,
    output wire        decode_error
);

    integer i;
    reg [7:0] received_label;

    always @(*) begin
        received_label = rx_word[7:0];

        for (i = 0; i < 8; i = i + 1)
            decoded_label[i] = received_label[7-i];

        decoded_sdi  = rx_word[9:8];
        decoded_data = rx_word[28:10];
        decoded_ssm  = rx_word[30:29];

        // Odd parity is valid when XOR reduction of all 32 bits is 1.
        parity_ok = ^rx_word;
    end

    assign decode_valid = rx_valid && parity_ok;
    assign decode_error = rx_valid && !parity_ok;

endmodule


module arinc429_transmitter (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        tx_start,
    input  wire        rate_select, // 1 = 100 kbps, 0 = 12.5 kbps
    input  wire [31:0] data_word,
    output reg  [1:0]  tx_out,      // 01 = positive, 10 = negative, 00 = null
    output reg         tx_busy,
    output reg         tx_done
);

    localparam IDLE      = 2'b00;
    localparam DATA_POS  = 2'b01;
    localparam DATA_ZERO = 2'b10;
    localparam GAP       = 2'b11;

    reg [1:0] state;
    reg [31:0] shift_reg;
    reg [5:0] bit_index;
    reg [6:0] phase_count;
    reg [8:0] gap_count;
    reg [5:0] half_period;
    reg [8:0] gap_limit;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state       <= IDLE;
            shift_reg   <= 32'b0;
            bit_index   <= 0;
            phase_count <= 0;
            gap_count   <= 0;
            half_period <= 5;
            gap_limit   <= 40;
            tx_out      <= 2'b00;
            tx_busy     <= 1'b0;
            tx_done     <= 1'b0;
        end else begin
            tx_done <= 1'b0;

            case (state)
                IDLE: begin
                    tx_out  <= 2'b00;
                    tx_busy <= 1'b0;

                    if (tx_start) begin
                        shift_reg   <= data_word;
                        bit_index   <= 0;
                        phase_count <= 0;

                        if (rate_select) begin
                            half_period <= 6'd5;
                            gap_limit   <= 9'd40;
                        end else begin
                            half_period <= 6'd40;
                            gap_limit   <= 9'd320;
                        end

                        tx_out  <= data_word[0] ? 2'b01 : 2'b10;
                        tx_busy <= 1'b1;
                        state   <= DATA_POS;
                    end
                end

                DATA_POS: begin
                    if (phase_count == half_period - 1'b1) begin
                        phase_count <= 0;
                        tx_out      <= 2'b00;
                        state       <= DATA_ZERO;
                    end else begin
                        phase_count <= phase_count + 1'b1;
                    end
                end

                DATA_ZERO: begin
                    if (phase_count == half_period - 1'b1) begin
                        phase_count <= 0;

                        if (bit_index == 6'd31) begin
                            tx_out    <= 2'b00;
                            gap_count <= 0;
                            state     <= GAP;
                        end else begin
                            shift_reg <= {1'b0, shift_reg[31:1]};
                            bit_index <= bit_index + 1'b1;
                            tx_out    <= shift_reg[1] ? 2'b01 : 2'b10;
                            state     <= DATA_POS;
                        end
                    end else begin
                        phase_count <= phase_count + 1'b1;
                    end
                end

                GAP: begin
                    tx_out <= 2'b00;

                    if (gap_count == gap_limit - 1'b1) begin
                        gap_count <= 0;
                        tx_done   <= 1'b1;
                        tx_busy   <= 1'b0;
                        state     <= IDLE;
                    end else begin
                        gap_count <= gap_count + 1'b1;
                    end
                end

                default: begin
                    state   <= IDLE;
                    tx_out  <= 2'b00;
                    tx_busy <= 1'b0;
                end
            endcase
        end
    end

endmodule


module arinc429_encoder (
    input  wire [7:0]  label,
    input  wire [1:0]  sdi,
    input  wire [18:0] data,
    input  wire [1:0]  ssm,
    output reg  [31:0] arinc_word,
    output reg         parity
);

    integer i;
    reg [7:0] reversed_label;

    always @(*) begin
        // ARINC 429 transmits the label MSB-first within the label field.
        for (i = 0; i < 8; i = i + 1)
            reversed_label[i] = label[7-i];

        arinc_word[7:0]   = reversed_label;
        arinc_word[9:8]   = sdi;
        arinc_word[28:10] = data;
        arinc_word[30:29] = ssm;

        // Odd parity across all 32 bits.
        parity = ~(^arinc_word[30:0]);
        arinc_word[31] = parity;
    end

endmodule
