#include <PWM/pwm.h>
#include "driver/ledc.h"

#define PWM_TIMER              LEDC_TIMER_0
#define PWM_MODE               LEDC_HIGH_SPEED_MODE
#define PWM_CLK                LEDC_AUTO_CLK
#define PWM_CHANNEL            LEDC_CHANNEL_0

enum PWM_State ePWM_State = PWM_OFF;

uint16_t duty_cycle = 0;

void PWM_init(uint8_t duty_resolution, uint8_t output_pin ,uint32_t frequency)
{
    ledc_timer_config_t pwm_timer = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = duty_resolution,
        .timer_num        = PWM_TIMER,
        .freq_hz          = frequency,  
        .clk_cfg          = PWM_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .intr_type      = 0,
        .gpio_num       = output_pin,
        .duty           = 0, 
        .hpoint         = 0
    };
    ledc_channel_config(&pwm_channel);
}

void PWM_control(enum PWM_State ePWM_State_control)
{
    ePWM_State = ePWM_State_control;

    PWM_set_duty_cycle(duty_cycle);
}

enum PWM_State PWM_get_state()
{
    return ePWM_State;
}

uint16_t PWM_get_duty_cycle()
{
    return duty_cycle;
}

void PWM_set_duty_cycle(uint16_t PWM_duty_cycle)
{
    duty_cycle = PWM_duty_cycle;

    switch (ePWM_State)
    {
    case PWM_ON:
        ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty_cycle);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL);

        break;

    case PWM_OFF:
        ledc_set_duty(PWM_MODE, PWM_CHANNEL, 0);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL);
        break;
    
    default:
        ledc_set_duty(PWM_MODE, PWM_CHANNEL, 0);
        ledc_update_duty(PWM_MODE, PWM_CHANNEL);
        break;
    }
}