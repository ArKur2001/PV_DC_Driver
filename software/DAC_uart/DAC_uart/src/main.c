#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "driver/uart.h"
#include "driver/dac_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_PORT UART_NUM_0

void app_main(void)
{
    //DAC
    dac_oneshot_handle_t dac;
    dac_oneshot_config_t cfg = {
        .chan_id = DAC_CHAN_0
    };
    dac_oneshot_new_channel(&cfg, &dac);

    //UART
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);

    char line[32];
    int pos = 0;
    uint8_t ch;

    while (1) 
    {
        int len = uart_read_bytes(UART_PORT, &ch, 1, portMAX_DELAY);

        if (len > 0) 
        {
            if (ch == '\n' || ch == '\r') 
            {
                line[pos] = 0;

                if (pos > 0) 
                {
                    int value = atoi(line);

                    if (value >= 0 && value <= 255) 
                    {
                        dac_oneshot_output_voltage(dac, value);
                        printf("%d\n", value);
                    } 
                }

                pos = 0;
            }
            else if (ch == 0x08 || ch == 0x7F) 
            {
                if (pos > 0) 
                {
                    pos--;
                }
            }
            else 
            {
                if (pos < sizeof(line) - 1) 
                {
                    line[pos++] = ch;
                }
            }
        }
    }
}