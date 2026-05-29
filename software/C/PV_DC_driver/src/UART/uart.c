#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "driver/uart.h"

#define UART_PORT UART_NUM_0
#define RX_BUFFER_SIZE 128

#define BUF_SIZE 1024

void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(
        UART_PORT,
        BUF_SIZE,
        0,
        0,
        NULL,
        0
    );

    uart_param_config(
        UART_PORT,
        &uart_config
    );

    // UART0 = USB serial
    uart_set_pin(
        UART_PORT,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );

    uart_flush(UART_PORT);
}

bool start_measurement = false;

bool uart_command_receiver(void)
{
    static char cmd_buffer[64];
    static int index = 0;

    uint8_t byte;

    // czytaj po 1 bajcie (non-blocking)
    int len = uart_read_bytes(
        UART_PORT,
        &byte,
        1,
        0
    );

    if (len > 0)
    {
        // koniec linii
        if (byte == '\n' || byte == '\r')
        {
            cmd_buffer[index] = '\0';

            // porównanie komendy
            if (strcmp(cmd_buffer, "start") == 0)
            {
                start_measurement = true;
                printf("START received\n");
            }

            // wyczyść bufor
            index = 0;
        }
        else
        {
            // zapis znaku jeśli jest miejsce
            if (index < sizeof(cmd_buffer) - 1)
            {
                cmd_buffer[index++] = (char)byte;
            }
            else
            {
                // overflow -> reset
                index = 0;
            }
        }
    }
    return start_measurement;
}