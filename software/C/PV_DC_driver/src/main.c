#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "inttypes.h"
#include "TASK_USER/task_user.h"
#include "TASK_CONTROL/task_control.h"
#include "TASK_MEMORY/task_memory.h"
#include "TASK_MPPT/task_mppt.h"
#include "BUTTONS/buttons.h"
#include "LCD/HD44780.h"
#include "LCD/LCD_string.h"
#include "LED/led.h"
#include <PWM/pwm.h>
#include "MEMORY/memory.h"
#include <ADC/adc.h>
#include <MEASUREMENTS/measurements.h>
#include "data_structures.h"
#include "esp_timer.h"

#define PWM_DUTY_RES_BIT            7       //128
#define PWM_DUTY_RES                128
#define PWM_PIN                     23      //GPIO23
#define PWM_FREQUENCY               10000

#define ADC_UNIT                    0       //ADC_UNIT_1
#define ADC_BITWIDTH                12      //ADC_BITWIDTH_12
#define ADC_ATTEN                   3       //ADC_ATTEN_DB_11
#define ADC_CURRENT_PIN             5       //ADC_CHANNEL_5(GPIO33)
#define ADC_VOLTAGE_PIN             4       //ADC_CHANNEL_5(GPIO32)

#define BUTTON_0_GPIO               14      //GPIO14  
#define BUTTON_1_GPIO               27      //GPIO27
#define BUTTON_2_GPIO               26      //GPIO26

#define VOLTAGE_MULTIPLIER          189.86  // 19.88   //voltage_divider_value
#define VOLTAGE_REF_LVL             900     //voltage_potentiometer_ref(mV)
#define CURRENT_REF_LVL             900     //current_potentiometer_ref(mV)

#define LCD_ADDR                    0x27
#define SDA_PIN                     21      //GPIO21
#define SCL_PIN                     22      //GPIO22
#define LCD_COLS                    20
#define LCD_ROWS                    4

#define DS18B20_BOILER_PIN          4       //boiler temperature sensor
#define DS18B20_CASE_PIN            5       //case temperature sensor

#define LED_GREEN_PIN               18      //GPIO18
#define LED_RED_PIN                 19      //GPIO19

QueueHandle_t BoilerSettings_queue;
QueueHandle_t ElectricalMeasurements_queue; 
QueueHandle_t TemperatureReadings_queue;
QueueHandle_t TimerData_queue;
QueueHandle_t MPPTData_queue;

void timer_callback(void *param)
{
    TimerData TimerData_data;
    ElectricalMeasurements ElectricalMeasurements_data;

    if (xQueuePeek(TimerData_queue, &TimerData_data, 0) == pdTRUE && xQueuePeek(ElectricalMeasurements_queue, &ElectricalMeasurements_data, 0)) 
    {
        TimerData_data.second_number++;
        TimerData_data.energy_j += ElectricalMeasurements_data.power_value;

        //printf("second_number = %" PRIu64 "\n", TimerData_data.second_number);
        //printf("energy = %" PRIu64 " J\n", TimerData_data.energy_j);

        xQueueOverwrite(TimerData_queue, &TimerData_data);
    } 
}

void queue_init()
{ 
    uint8_t  desired_temperature;
    uint16_t boiler_capacity;
    uint64_t energy_j;

    flash_read(&desired_temperature, &boiler_capacity, &energy_j);

    BoilerSettings BoilerSettings_data = {desired_temperature, boiler_capacity};
    ElectricalMeasurements ElectricalMeasurements_data = {100.0, 10.0, 1000.0};
    TemperatureReadings TemperatureReadings_data = {50.0, 50.0};
    TimerData TimerData_data = {0, energy_j};
    MPPTData MPPTData_data = {1000.0, MPPT_NOT_ALLOWED};
    
    xQueueSend(BoilerSettings_queue, &BoilerSettings_data, pdMS_TO_TICKS(100));
    xQueueSend(ElectricalMeasurements_queue, &ElectricalMeasurements_data, pdMS_TO_TICKS(100));
    xQueueSend(TemperatureReadings_queue, &TemperatureReadings_data, pdMS_TO_TICKS(100));
    xQueueSend(TimerData_queue, &TimerData_data, pdMS_TO_TICKS(100));
    xQueueSend(MPPTData_queue, &MPPTData_data, pdMS_TO_TICKS(100));
}

void test_receive()
{
    static TemperatureReadings boiler_data;
    static MPPTData mppt_data;
    
    while(1)
    {
        xQueuePeek(TemperatureReadings_queue, &boiler_data, pdMS_TO_TICKS(0));
        xQueuePeek(MPPTData_queue, &mppt_data, pdMS_TO_TICKS(0));

        if(mppt_data.eMPPT_Permission == MPPT_ALLOWED)
        {
            printf("allowed\n");
        }
        else
        {
            //printf("not allowed\n");
        }

        //printf("boiler temp is: %f\n", boiler_data.temp_water);
        //printf("case temp is: %f\n",  boiler_data.temp_case);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() 
{
    Button_Init(BUTTON_0_GPIO, BUTTON_1_GPIO, BUTTON_2_GPIO);
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    Led_Init(LED_GREEN_PIN, LED_RED_PIN);
    PWM_init(PWM_DUTY_RES_BIT, PWM_PIN, PWM_FREQUENCY);
    ADC1_init(ADC_UNIT, ADC_BITWIDTH, ADC_ATTEN);
    set_adc_pin(ADC_CURRENT_PIN, ADC_VOLTAGE_PIN);
    measurements_init(VOLTAGE_REF_LVL, VOLTAGE_MULTIPLIER, CURRENT_REF_LVL);
   
    const esp_timer_create_args_t program_timer_args = {.callback = &timer_callback, .name = "Program_timer"};

    esp_timer_handle_t program_timer_handler;
    esp_timer_create(&program_timer_args, &program_timer_handler);
    esp_timer_start_periodic(program_timer_handler, 1000000);

    LCD_INFO_state();

    BoilerSettings_queue = xQueueCreate(1, sizeof(BoilerSettings));
    ElectricalMeasurements_queue = xQueueCreate(1, sizeof(ElectricalMeasurements));
    TemperatureReadings_queue = xQueueCreate(1, sizeof(TemperatureReadings));
    TimerData_queue = xQueueCreate(1, sizeof(TimerData));
    MPPTData_queue = xQueueCreate(1, sizeof(MPPTData));

    queue_init();

    TaskUserParameters Task_User_params = {BoilerSettings_queue, ElectricalMeasurements_queue, TemperatureReadings_queue, TimerData_queue};
    TaskControlParameters Task_Control_params = {BoilerSettings_queue, ElectricalMeasurements_queue, TemperatureReadings_queue, TimerData_queue, MPPTData_queue, DS18B20_BOILER_PIN, DS18B20_CASE_PIN};
    TaskMemoryParameters Task_Memory_params = {BoilerSettings_queue, TimerData_queue};
    TaskMPPTParameters Task_MPPT_params = {ElectricalMeasurements_queue, MPPTData_queue, ADC_VOLTAGE_PIN, ADC_CURRENT_PIN, PWM_DUTY_RES_BIT};

    xTaskCreate(Task_User, "Task_User", 4096, &Task_User_params, 20, NULL);
    xTaskCreate(Task_Control, "Task_Control", 4096, &Task_Control_params, 30, NULL);
    xTaskCreate(Task_Memory, "Task_Memory", 4096, &Task_Memory_params, 10, NULL);
    xTaskCreate(Task_MPPT, "Task_MPPT", 4096, &Task_MPPT_params, 20, NULL);
    xTaskCreate(test_receive, "Task_Receive", 4096, NULL, 10, NULL);
}