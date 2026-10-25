#ifndef HAL_H
#define HAL_H

#include <stdint.h>

#define HAL_ADC_RESOLUTION  1024
#define HAL_ADC_VREF        3.3f
#define HAL_ADC_CHANNELS    4

#define HAL_PWM_RESOLUTION  256
#define HAL_PWM_CHANNELS    2

#define HAL_GPIO_PINS       8

typedef struct {
    float    adc_voltages[HAL_ADC_CHANNELS];
    uint16_t adc_raw[HAL_ADC_CHANNELS];

    uint8_t  pwm_duty[HAL_PWM_CHANNELS];
    float    pwm_voltage_scale;

    uint32_t tick_us;
    uint32_t timer_period_us;
    uint8_t  timer_flag;

    uint8_t  gpio_state[HAL_GPIO_PINS];
} HAL;

void hal_init(HAL *hal);
void hal_adc_set_voltage(HAL *hal, uint8_t channel, float voltage);
uint16_t hal_adc_read(const HAL *hal, uint8_t channel);
float hal_adc_to_voltage(uint16_t raw);
void hal_pwm_write(HAL *hal, uint8_t channel, uint8_t duty);
float hal_pwm_get_voltage(const HAL *hal, uint8_t channel);
void hal_timer_tick(HAL *hal, uint32_t dt_us);
uint8_t hal_timer_check(HAL *hal);
void hal_timer_set_period(HAL *hal, uint32_t period_us);
void hal_gpio_write(HAL *hal, uint8_t pin, uint8_t state);
uint8_t hal_gpio_read(const HAL *hal, uint8_t pin);

#endif
