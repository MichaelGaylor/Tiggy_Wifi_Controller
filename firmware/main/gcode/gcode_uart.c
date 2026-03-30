/*
 * WiFi CNC Controller - G-Code UART Handler
 *
 * Receives G-code over USB serial (UART0). Processes bytes one at a time,
 * checking for real-time commands and accumulating lines for the parser.
 */

#include "gcode_uart.h"
#include "gcode_interface.h"
#include "gcode_parser.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gcode_uart";

#define UART_PORT       UART_NUM_0
#define UART_BAUD       115200
#define UART_BUF_SIZE   256

static volatile bool s_active;

/* Output callback: send to UART */
static void uart_output(const char *str, size_t len, void *ctx)
{
    (void)ctx;
    uart_write_bytes(UART_PORT, str, len);
}

void gcode_uart_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UART G-code handler starting on UART%d @ %d baud",
             UART_PORT, UART_BAUD);

    /* Configure UART (uses default GPIO for UART0: TX=43, RX=44 on S3) */
    uart_config_t uart_config = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2,
                        0, NULL, 0);

    /* Set output to UART */
    gcode_interface_set_output(uart_output, NULL);

    /* Send welcome */
    const char *welcome = "\r\nGrbl 1.1h ['$' for help]\r\n";
    uart_write_bytes(UART_PORT, welcome, strlen(welcome));

    char line_buf[GCODE_MAX_LINE_LENGTH];
    int line_pos = 0;
    uint8_t rx_buf[64];

    s_active = true;

    while (1) {
        int n = uart_read_bytes(UART_PORT, rx_buf, sizeof(rx_buf),
                                pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            uint8_t byte = rx_buf[i];

            /* Real-time commands — handled immediately */
            if (byte == GRBL_RT_STATUS_QUERY ||
                byte == GRBL_RT_FEED_HOLD ||
                byte == GRBL_RT_CYCLE_START ||
                byte == GRBL_RT_SOFT_RESET) {
                gcode_handle_realtime(byte);
                continue;
            }

            /* Line accumulation */
            if (byte == '\n' || byte == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    gcode_interface_submit_line(line_buf);
                    line_pos = 0;
                }
            } else if (byte == 0x08 || byte == 0x7F) {
                /* Backspace / delete */
                if (line_pos > 0) line_pos--;
            } else if (line_pos < GCODE_MAX_LINE_LENGTH - 1) {
                line_buf[line_pos++] = (char)byte;
            }
        }
    }
}

bool gcode_uart_active(void)
{
    return s_active;
}
