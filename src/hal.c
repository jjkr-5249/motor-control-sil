#include "hal.h"
#include <string.h>

#define HAL_DEFAULT_CONTROL_PERIOD_US  1000

void hal_init(HAL *hal)
{
    memset(hal, 0, sizeof(HAL));
    hal->pwm_voltage_scale = 12.0f;
    hal->timer_period_us = HAL_DEFAULT_CONTROL_PERIOD_US;
}

void hal_adc_set_voltage(HAL *hal, uint8_t channel, float voltage)
{
    if (channel >= HAL_ADC_CHANNELS) return;

    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > HAL_ADC_VREF) voltage = HAL_ADC_VREF;

    hal->adc_voltages[channel] = voltage;
    hal->adc_raw[channel] = (uint16_t)((voltage / HAL_ADC_VREF)
                            * (HAL_ADC_RESOLUTION - 1));
}

uint16_t hal_adc_read(const HAL *hal, uint8_t channel)
{
    if (channel >= HAL_ADC_CHANNELS) return 0;
    return hal->adc_raw[channel];
}

float hal_adc_to_voltage(uint16_t raw)
{
    return ((float)raw / (HAL_ADC_RESOLUTION - 1)) * HAL_ADC_VREF;
}

void hal_pwm_write(HAL *hal, uint8_t channel, uint8_t duty)
{
    if (channel >= HAL_PWM_CHANNELS) return;
    hal->pwm_duty[channel] = duty;
}

float hal_pwm_get_voltage(const HAL *hal, uint8_t channel)
{
    if (channel >= HAL_PWM_CHANNELS) return 0.0f;
    return ((float)hal->pwm_duty[channel] / (HAL_PWM_RESOLUTION - 1))
           * hal->pwm_voltage_scale;
}

void hal_timer_tick(HAL *hal, uint32_t dt_us)
{
    hal->tick_us += dt_us;

    if (hal->tick_us >= hal->timer_period_us) {
        hal->timer_flag = 1;
        hal->tick_us -= hal->timer_period_us;
    }
}

uint8_t hal_timer_check(HAL *hal)
{
    if (hal->timer_flag) {
        hal->timer_flag = 0;
        return 1;
    }
    return 0;
}

void hal_timer_set_period(HAL *hal, uint32_t period_us)
{
    hal->timer_period_us = period_us;
}

void hal_gpio_write(HAL *hal, uint8_t pin, uint8_t state)
{
    if (pin >= HAL_GPIO_PINS) return;
    hal->gpio_state[pin] = state ? 1 : 0;
}

uint8_t hal_gpio_read(const HAL *hal, uint8_t pin)
{
    if (pin >= HAL_GPIO_PINS) return 0;
    return hal->gpio_state[pin];
}
