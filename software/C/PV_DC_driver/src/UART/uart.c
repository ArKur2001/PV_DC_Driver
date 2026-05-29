#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "driver/uart.h"

#define UART_PORT UART_NUM_0
#define RX_BUFFER_SIZE 128

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