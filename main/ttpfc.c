#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_filter.h"

#define V_BUS_REF 400.0 // Desired output voltage (in volts)
#define V_AC 220.0      // normal AC voltage
#define I_BUS_REF 12.5  // DESIRED OUTPUT CURRENT (in Amps)
#define PWM_FREQ 50000  // 50 kHz PWM frequency

// GPIO pins for MCPWM
#define MCPWM_GPIO_PWM0A GPIO_NUM_18
#define MCPWM_GPIO_PWM0B GPIO_NUM_19

#define ADC_FILTER_LENGTH 12 // Length of the moving average filter

/*############################################################################################################################
                                                Software Filtering  ADC
############################################################################################################################ */

// Filter variables
float input_voltage_filter[ADC_FILTER_LENGTH] = {0};
float output_voltage_filter[ADC_FILTER_LENGTH] = {0};
float input_current_filter[ADC_FILTER_LENGTH] = {0};
float output_current_filter[ADC_FILTER_LENGTH] = {0};

// Function to calculate the moving average
float moving_average_filter(float *buffer, float new_sample, int length)
{
    float sum = 0.0;

    // Shift the buffer
    for (int i = length - 1; i > 0; i--)
    {
        buffer[i] = buffer[i - 1];
    }
    buffer[0] = new_sample;

    // Compute the average
    for (int i = 0; i < length; i++)
    {
        sum += buffer[i];
    }
    return sum / length;
}

/*############################################################################################################################
                                                Software Filtering  ADC
############################################################################################################################ */

void initialize_mcpwm(void)
{
    mcpwm_config_t pwm_config = {
        .frequency = PWM_FREQ,
        .cmpr_a = 0,                    // Initial duty cycle for PWMxA
        .cmpr_b = 0,                    // Initial duty cycle for PWMxB
        .duty_mode = MCPWM_DUTY_MODE_0, // Active HIGH
        .counter_mode = MCPWM_UP_COUNTER,
    };

    // Initialize MCPWM unit 0, timer 0
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // Set dead time for complementary signals
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 50, 50); // 50ns dead time
}

// initalizing MCPWM for PWM generation
void initialize_mcpwm_gpio_init(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MCPWM_GPIO_PWM0A);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MCPWM_GPIO_PWM0B);
}

void initialize_adc(void)
{
    // Configure ADC channels
    adc1_config_width(ADC_WIDTH_BIT_12);                        // 12-bit resolution
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // Input voltage sense, GPIO36
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11); // Output voltage sense, GPIO39
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // INPUT Current sense, GPIO34
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); // OUTPUT Current sense, GPIO32
}

void initialize_gpio(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_4), // Fault indicator
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

/*############################################################################################################################
                                                Hardware Filtering  ADC
############################################################################################################################ */
/*
// Hardware filter initialization (commented to avoid conflicts with software filter)

void initialize_adc_with_filter(void)
{
    // Configure ADC filter for input voltage
    adc_filter_config_t filter_config_voltage = {
        .unit = ADC_UNIT_1,
        .channel = ADC_CHANNEL_0, // Input voltage sense
        .mode = ADC_FILTER_MODE_MOVING_AVERAGE,
        .len = ADC_FILTER_LENGTH, // Length of the moving average
    };
    adc_filter_enable(&filter_config_voltage);

    // Configure ADC filter for output voltage
    adc_filter_config_t filter_config_output_voltage = {
        .unit = ADC_UNIT_1,
        .channel = ADC_CHANNEL_3, // Output voltage sense
        .mode = ADC_FILTER_MODE_MOVING_AVERAGE,
        .len = ADC_FILTER_LENGTH,
    };
    adc_filter_enable(&filter_config_output_voltage);

    // Configure ADC filter for input current
    adc_filter_config_t filter_config_input_current = {
        .unit = ADC_UNIT_1,
        .channel = ADC_CHANNEL_6, // Input current sense
        .mode = ADC_FILTER_MODE_MOVING_AVERAGE,
        .len = ADC_FILTER_LENGTH,
    };
    adc_filter_enable(&filter_config_input_current);

    // Configure ADC filter for output current
    adc_filter_config_t filter_config_output_current = {
        .unit = ADC_UNIT_1,
        .channel = ADC_CHANNEL_4, // Output current sense
        .mode = ADC_FILTER_MODE_MOVING_AVERAGE,
        .len = ADC_FILTER_LENGTH,
    };
    adc_filter_enable(&filter_config_output_current);
}

*/
/*############################################################################################################################
                                                Hardware Filtering  ADC
############################################################################################################################ */

void control_task(void *arg)
{
    float output_voltage = 0.0;
    float input_voltage = 0.0;
    float output_current = 0.0;
    float input_current = 0.0;

    // float ki_gain = 0.01;
    // float kv_gain = 0.001;
    // float kvac_gain = 0.001;
    // float kv_fltr = 0.001;

    float kp_v = 0.1, ki_v = 0.05; // Voltage loop PI gains
    float SAMPLE_TIME = 0.001;     // sample time (1ms)

    float kp_c = 0.2, ki_c = 0.02; // Current loop PI gains

    float voltage_error = 0.0, current_error = 0.0;
    float prev_voltage_error = 0.0;
    float prev_current_error = 0.0;

    float voltage_integral = 0.0, current_integral = 0.0;
    float voltage_proportional = 0.0, current_proportional = 0.0;

    float prev_voltage_integral = 0.0, prev_current_integral = 0.0;

    float duty_cycle = 0.0;

    float current_ref = 0.0; // current reference for inner loop

    while (1)
    {

        /*############################################################################################################################
                                                        Software Filtering  ADC
        ############################################################################################################################ */

        // Read ADC values
        float raw_input_voltage = adc1_get_raw(ADC1_CHANNEL_0) * (3.3 / 4095.0) * (V_AC / 3.3);
        float raw_output_voltage = adc1_get_raw(ADC1_CHANNEL_3) * (3.3 / 4095.0) * (V_BUS_REF / 3.3);
        float raw_output_current = adc1_get_raw(ADC1_CHANNEL_4) * (3.3 / 4095.0) * (I_BUS_REF / 3.3);
        float raw_input_current = adc1_get_raw(ADC1_CHANNEL_6) * (3.3 / 4095.0) * (I_BUS_REF / 3.3);

        // Apply software filters
        input_voltage = moving_average_filter(input_voltage_filter, raw_input_voltage, ADC_FILTER_LENGTH);
        output_voltage = moving_average_filter(output_voltage_filter, raw_output_voltage, ADC_FILTER_LENGTH);
        output_current = moving_average_filter(output_current_filter, raw_output_current, ADC_FILTER_LENGTH);
        input_current = moving_average_filter(input_current_filter, raw_input_current, ADC_FILTER_LENGTH);

        /*############################################################################################################################
                                                        Software Filtering  ADC
        ############################################################################################################################ */

        /*############################################################################################################################
                                                        Hardware Filtering  ADC
        ############################################################################################################################ */
        /*
         input_voltage = adc_filter_get_average(ADC_UNIT_1, ADC_CHANNEL_0) * (3.3 / 4095.0) * (V_AC / 3.3);
         output_voltage = adc_filter_get_average(ADC_UNIT_1, ADC_CHANNEL_3) * (3.3 / 4095.0) * (V_BUS_REF / 3.3);
         input_current = adc_filter_get_average(ADC_UNIT_1, ADC_CHANNEL_6) * (3.3 / 4095.0) * (I_BUS_REF / 3.3);
         output_current = adc_filter_get_average(ADC_UNIT_1, ADC_CHANNEL_4) * (3.3 / 4095.0) * (I_BUS_REF / 3.3);

 */
        /*############################################################################################################################
                                                        Hardware Filtering  ADC
        ############################################################################################################################ */

        /*############################################################################################################################
                        Voltage and Current Closed Control loop using PID, Discretization using Bilinear Transformation
        ############################################################################################################################ */

        // Voltage Control Loop
        voltage_error = V_BUS_REF - output_voltage; // voltage error calculate

        voltage_proportional = kp_v * voltage_error;
        voltage_integral = prev_voltage_integral + ki_v * 0.5 * (voltage_error + prev_voltage_error) * SAMPLE_TIME; // update integral term
        prev_voltage_error = voltage_error;
        prev_voltage_integral = voltage_integral;

        // Anti-windup for voltage integrator
        if ((voltage_proportional + prev_voltage_integral) > I_BUS_REF)
        {
            prev_voltage_integral = I_BUS_REF - voltage_proportional;
        }
        else if ((voltage_proportional + prev_voltage_integral) < 0)
        {
            prev_voltage_integral = -voltage_proportional;
        }

        // Current Control Loop
        current_ref = voltage_proportional + prev_voltage_integral; // compute current reference from voltage PI controller

        // Clamp current reference to a maximum allowable value
        if (current_ref > I_BUS_REF)
        {
            current_ref = I_BUS_REF;
        }
        else if (current_ref < 0)
        {
            current_ref = 0;
        }

        // Current control loop (Inner loop)
        current_error = current_ref - output_current;

        current_proportional = kp_c * current_error;
        current_integral = prev_current_integral + ki_c * 0.5 * (current_error + prev_current_error) * SAMPLE_TIME;
        prev_current_error = current_error;
        prev_current_integral = current_integral;

        // Anti-windup for current integrator
        if ((current_proportional + prev_current_integral) > 100.0)
        {
            prev_current_integral = 100.0 - current_proportional;
        }
        else if ((current_proportional + prev_current_integral) < 0)
        {
            prev_current_integral = -current_proportional;
        }

        duty_cycle = current_proportional + prev_current_integral;

        // Clamp duty cycle
        if (duty_cycle > 100.0)
            duty_cycle = 100.0;
        if (duty_cycle < 0.0)
            duty_cycle = 0.0;

        // Update PWM duty cycle
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 100.0 - duty_cycle);

        vTaskDelay(pdMS_TO_TICKS((int)(SAMPLE_TIME * 1000))); // waiting for next sample time

        /*############################################################################################################################
                        Voltage and Current Closed Control loop using PID, Discretization using Bilinear Transformation
        ############################################################################################################################ */
    }
}

void app_main(void)
{
    initialize_mcpwm();
    initialize_mcpwm_gpio_init();
    initialize_adc(); // for software ADC filtering
    initialize_gpio();
    // initialize_adc_with_filter();       //for hardware ADC filtering

    // Create control loop task
    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
}