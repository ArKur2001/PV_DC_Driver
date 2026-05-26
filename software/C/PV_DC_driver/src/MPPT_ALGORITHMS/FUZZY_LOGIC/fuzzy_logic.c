#include "inttypes.h"
#include "fuzzy_logic.h"
#include "TASK_MPPT/task_mppt.h"
#include "PWM/pwm.h"
#include "data_structures.h"
#include "math.h"

#define DUTY_MULTIPLIER 2.0

enum MPPT_stage {SETUP, SET_DUTY};

void calculate_membership(double data, double step_size, double out[5])
{
    double array[5];

    array[0] = -(step_size * 2.0);
    array[1] = -step_size;
    array[2] = 0.0;
    array[3] = step_size;
    array[4] = step_size * 2.0;

    for (int i = 0; i < 5; i++) {
        out[i] = 0.0;
    }

    // NB
    if (data <= array[0]) {
        out[0] = 1.0;
    }
    else if ((array[0] < data) && (data <= array[1])) {
        out[0] = (array[1] - data) / (array[1] - array[0]);
    }

    // NS
    if (data <= array[0]) {
        out[1] = 0.0;
    }
    else if ((array[0] < data) && (data <= array[1])) {
        out[1] = (data - array[0]) / (array[1] - array[0]);
    }
    else if ((array[1] < data) && (data <= array[2])) {
        out[1] = (array[2] - data) / (array[2] - array[1]);
    }

    // ZO
    if (data <= array[1]) {
        out[2] = 0.0;
    }
    else if ((array[1] < data) && (data <= array[2])) {
        out[2] = (data - array[1]) / (array[2] - array[1]);
    }
    else if ((array[2] < data) && (data <= array[3])) {
        out[2] = (array[3] - data) / (array[3] - array[2]);
    }

    // PS
    if (data <= array[2]) {
        out[3] = 0.0;
    }
    else if ((array[2] < data) && (data <= array[3])) {
        out[3] = (data - array[2]) / (array[3] - array[2]);
    }
    else if ((array[3] < data) && (data <= array[4])) {
        out[3] = (array[4] - data) / (array[4] - array[3]);
    }

    // PB
    if (data <= array[3]) {
        out[4] = 0.0;
    }
    else if ((array[3] < data) && (data <= array[4])) {
        out[4] = (data - array[3]) / (array[4] - array[3]);
    }
    else {
        out[4] = 1.0;
    }
}

void Fuzzy_logic_algorithm(MPPTData *MPPTData_data, ElectricalMeasurements ElectricalMeasurements_data, uint8_t pwm_duty_resolution_bit, enum Task_MPPT_state *eTask_MPPT_state, enum Algorithm_Status *eAlgorithm_Status)
{
    static enum MPPT_stage eMPPT_stage = SETUP;

    double rule_table[5][5] = 
    {
        { 0.0,   0.0,   -0.04, -0.04, -0.04 },
        { 0.0,   0.0,   -0.02, -0.02, -0.02 },
        { -0.02, 0.0,    0.0,   0.0,   0.02 },
        { 0.02,  0.02,   0.02,  0.0,   0.0  },
        { 0.04,  0.04,   0.04,  0.0,   0.0  }
    };

    uint16_t duty_min = 1;
    uint16_t duty_max = pow(2, pwm_duty_resolution_bit) - 1;
    int16_t duty = 0;
    static float duty_buf = 0.0;

    static double power_prev = 0.0;
    static double e_prev = 0.0;

    switch (eMPPT_stage)
    {
        case SETUP:
            if(MPPTData_data->eMPPT_Permission == MPPT_NOT_ALLOWED)
            {
                eMPPT_stage = SETUP;
                *eTask_MPPT_state = SEND;
                *eAlgorithm_Status = ALGORITHM_DONE;
            }
            else
            {
                power_prev = 0.0;
                e_prev = 0.0;

                duty = pow(2, pwm_duty_resolution_bit - 1);
                duty_buf = 0.5;

                PWM_set_duty_cycle(duty);

                vTaskDelay(pdMS_TO_TICKS(1000));

                eMPPT_stage = SET_DUTY;
                *eTask_MPPT_state = MEASUREMENTS;
                *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            }

            break;
        
        case SET_DUTY:
            double E_buf = 0.0;
            double CE_buf = 0.0;

            E_buf = ElectricalMeasurements_data.power_value - power_prev;
            CE_buf = E_buf - e_prev;

            power_prev = ElectricalMeasurements_data.power_value;
            e_prev = E_buf;

            double E_membership[5] = {0.0};
            double CE_membership[5] = {0.0};

            calculate_membership(E_buf, 1.0, E_membership);
            calculate_membership(CE_buf, 0.6, CE_membership);

            double weight_duty_sum = 0.0;
            double weight_sum = 0.0;
            double activation = 0.0;
            float duty_delta = 0.0;

            int8_t i = 0;
            int8_t j = 0;

            for(i = 0 ; i < 5 ; i++)
            {
                for(j = 0 ; j < 5 ; j++)
                {
                    activation = fmin(E_membership[i], CE_membership[j]);

                    weight_duty_sum += activation * rule_table[i][j];
                    weight_sum += activation;
                }
            }

            if(weight_sum == 0.0)
            {
                duty_delta = 0.0;
            }
            else
            {
                duty_delta = (weight_duty_sum / weight_sum) * DUTY_MULTIPLIER;
            }

            duty_buf += duty_delta;

            if(duty_buf >= 1.0)
            {
                duty = duty_max;
            }
            else if(duty_buf <= 0.0)
            {
                duty = duty_min;
            }
            else
            {
                duty = duty_min + (uint16_t)(duty_buf * (duty_max - duty_min));
            }

            PWM_set_duty_cycle(duty);

            eMPPT_stage = SET_DUTY;
            *eTask_MPPT_state = SEND;
            *eAlgorithm_Status = ALGORITHM_NOT_DONE;
            
            break;

        default:
            eMPPT_stage = SETUP;

            break;
    }
}