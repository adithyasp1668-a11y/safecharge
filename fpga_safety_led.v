module fpga_safety_led (
    input  wire clk,
    input  wire uart_rx,
    output wire led
);

    // =========================================================
    // PYNQ-Z2 CLOCK
    // =========================================================
    // 125 MHz clock on H16
    // ESP32 UART = 115200 baud

    localparam integer BAUD_DIV = 1085;

    // =========================================================
    // SAFETY LIMITS
    // =========================================================

    localparam integer MAX_VOLTAGE = 5500;  // 5.5 V
    localparam integer MAX_CURRENT = 7500;  // 7.5 A
    localparam integer MAX_TEMP    = 4000;  // 40.0 C

    // =========================================================
    // UART SYNCHRONIZER
    // =========================================================

    reg rx_sync1 = 1'b1;
    reg rx_sync2 = 1'b1;

    always @(posedge clk) begin
        rx_sync1 <= uart_rx;
        rx_sync2 <= rx_sync1;
    end

    // =========================================================
    // UART RECEIVER
    // =========================================================

    localparam UART_IDLE  = 2'd0;
    localparam UART_START = 2'd1;
    localparam UART_DATA  = 2'd2;
    localparam UART_STOP  = 2'd3;

    reg [1:0] uart_state = UART_IDLE;

    reg [12:0] baud_counter = 13'd0;
    reg [2:0]  bit_count = 3'd0;

    reg [7:0] rx_byte = 8'd0;

    // =========================================================
    // PACKET PARSER
    //
    // Expected:
    // V=4200,I=850,T=2950,F=0,S=1
    //
    // V = voltage in mV
    // I = current in mA
    // T = temperature in centi-degrees C
    // F = ESP32 fault
    // S = sensor/system status
    // =========================================================

    localparam FIELD_NONE = 3'd0;
    localparam FIELD_V    = 3'd1;
    localparam FIELD_I    = 3'd2;
    localparam FIELD_T    = 3'd3;
    localparam FIELD_F    = 3'd4;
    localparam FIELD_S    = 3'd5;

    reg [2:0] current_field = FIELD_NONE;

    reg [31:0] number_value = 32'd0;

    reg reading_number = 1'b0;

    // =========================================================
    // STORED VALUES
    // =========================================================

    reg [31:0] voltage_value     = 32'd0;
    reg [31:0] current_value     = 32'd0;
    reg [31:0] temperature_value = 32'd0;

    reg esp_fault = 1'b0;
    reg sensor_ok = 1'b1;

    reg data_received = 1'b0;

    // =========================================================
    // UART RECEIVER + PARSER
    // =========================================================

    always @(posedge clk) begin

        case (uart_state)

            // -------------------------------------------------
            // IDLE
            // -------------------------------------------------

            UART_IDLE: begin

                baud_counter <= 0;
                bit_count <= 0;

                // UART start bit
                if (rx_sync2 == 1'b0) begin

                    uart_state <= UART_START;

                    // Check middle of start bit
                    baud_counter <= BAUD_DIV / 2;
                end

            end

            // -------------------------------------------------
            // START BIT
            // -------------------------------------------------

            UART_START: begin

                if (baud_counter == 0) begin

                    if (rx_sync2 == 1'b0) begin

                        uart_state <= UART_DATA;

                        bit_count <= 0;

                        baud_counter <= BAUD_DIV - 1;

                    end
                    else begin

                        uart_state <= UART_IDLE;

                    end

                end
                else begin

                    baud_counter <= baud_counter - 1'b1;

                end

            end

            // -------------------------------------------------
            // DATA BITS
            // -------------------------------------------------

            UART_DATA: begin

                if (baud_counter == 0) begin

                    rx_byte[bit_count] <= rx_sync2;

                    baud_counter <= BAUD_DIV - 1;

                    if (bit_count == 3'd7) begin

                        uart_state <= UART_STOP;

                    end
                    else begin

                        bit_count <= bit_count + 1'b1;

                    end

                end
                else begin

                    baud_counter <= baud_counter - 1'b1;

                end

            end

            // -------------------------------------------------
            // STOP BIT
            // -------------------------------------------------

            UART_STOP: begin

                if (baud_counter == 0) begin

                    if (rx_sync2 == 1'b1) begin

                        // =====================================
                        // FIELD NAME
                        // =====================================

                        if (rx_byte == "V") begin

                            current_field <= FIELD_V;
                            number_value <= 0;
                            reading_number <= 0;

                        end

                        else if (rx_byte == "I") begin

                            current_field <= FIELD_I;
                            number_value <= 0;
                            reading_number <= 0;

                        end

                        else if (rx_byte == "T") begin

                            current_field <= FIELD_T;
                            number_value <= 0;
                            reading_number <= 0;

                        end

                        else if (rx_byte == "F") begin

                            current_field <= FIELD_F;
                            number_value <= 0;
                            reading_number <= 0;

                        end

                        else if (rx_byte == "S") begin

                            current_field <= FIELD_S;
                            number_value <= 0;
                            reading_number <= 0;

                        end

                        // =====================================
                        // =
                        // =====================================

                        else if (rx_byte == "=") begin

                            number_value <= 0;
                            reading_number <= 1;

                        end

                        // =====================================
                        // DIGIT
                        // =====================================

                        else if ((rx_byte >= "0") &&
                                 (rx_byte <= "9")) begin

                            if (reading_number) begin

                                number_value <=
                                    (number_value * 10) +
                                    (rx_byte - "0");

                            end

                        end

                        // =====================================
                        // FIELD END
                        // =====================================

                        else if ((rx_byte == ",") ||
                                 (rx_byte == 8'h0A) ||
                                 (rx_byte == 8'h0D)) begin

                            if (reading_number) begin

                                // -----------------------------
                                // VOLTAGE
                                // -----------------------------

                                if (current_field == FIELD_V) begin

                                    voltage_value <= number_value;

                                end

                                // -----------------------------
                                // CURRENT
                                // -----------------------------

                                else if (current_field == FIELD_I) begin

                                    current_value <= number_value;

                                end

                                // -----------------------------
                                // TEMPERATURE
                                // -----------------------------

                                else if (current_field == FIELD_T) begin

                                    temperature_value <= number_value;

                                end

                                // -----------------------------
                                // ESP32 FAULT
                                // -----------------------------

                                else if (current_field == FIELD_F) begin

                                    if (number_value != 0)
                                        esp_fault <= 1'b1;
                                    else
                                        esp_fault <= 1'b0;

                                end

                                // -----------------------------
                                // SENSOR STATUS
                                // -----------------------------

                                else if (current_field == FIELD_S) begin

                                    if (number_value == 0)
                                        sensor_ok <= 1'b0;
                                    else
                                        sensor_ok <= 1'b1;

                                end

                                data_received <= 1'b1;

                            end

                            number_value <= 0;
                            reading_number <= 0;

                        end

                    end

                    uart_state <= UART_IDLE;
                    baud_counter <= 0;

                end
                else begin

                    baud_counter <= baud_counter - 1'b1;

                end

            end

            default: begin

                uart_state <= UART_IDLE;

            end

        endcase

    end

    // =========================================================
    // FPGA SAFETY DECISION
    // =========================================================

    wire voltage_fault;
    wire current_fault;
    wire temperature_fault;
    wire esp32_fault;
    wire sensor_fault;

    assign voltage_fault =
        (voltage_value > MAX_VOLTAGE);

    assign current_fault =
        (current_value > MAX_CURRENT);

    assign temperature_fault =
        (temperature_value > MAX_TEMP);

    assign esp32_fault =
        (esp_fault == 1'b1);

    assign sensor_fault =
        (sensor_ok == 1'b0);

    // =========================================================
    // LED0
    // =========================================================

    // LED turns ON when any safety fault occurs.
    // Before receiving data, LED remains OFF.

    assign led =
        data_received &&
        (
            voltage_fault ||
            current_fault ||
            temperature_fault ||
            esp32_fault ||
            sensor_fault
        );

endmodule