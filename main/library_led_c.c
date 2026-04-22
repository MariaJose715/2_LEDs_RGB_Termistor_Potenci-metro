#include "library_led_c.h"
#include <math.h>

void config_led_rgb(led_rgb_t *led_rgb)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = led_rgb->speed_mode,
        .duty_resolution  = led_rgb->duty_resolution,
        .timer_num        = led_rgb->timer,
        .freq_hz          = led_rgb->frequency,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_red = {
        .speed_mode     = led_rgb->speed_mode,
        .channel        = led_rgb->led_red.channel,
        .timer_sel      = led_rgb->timer,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = led_rgb->led_red.gpio_num,
        .duty           = led_rgb->led_red.duty, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red));
    // Prepare and then apply the LEDC PWM channel configuration for green LED
    ledc_channel_config_t ledc_channel_green = {
        .speed_mode     = led_rgb->speed_mode,
        .channel        = led_rgb->led_green.channel,
        .timer_sel      = led_rgb->timer,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = led_rgb->led_green.gpio_num,
        .duty           = led_rgb->led_green.duty, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green));
    // Prepare and then apply the LEDC PWM channel configuration for blue LED   
    ledc_channel_config_t ledc_channel_blue = {
        .speed_mode     = led_rgb->speed_mode,
        .channel        = led_rgb->led_blue.channel,
        .timer_sel      = led_rgb->timer,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = led_rgb->led_blue.gpio_num,
        .duty           = led_rgb->led_blue.duty, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_blue));
}

void set_led_rgb_given_struct(led_rgb_t *led_rgb)
{
    // Set duty for red LED
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_red.channel, led_rgb->led_red.duty));
    // Set duty for green LED
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_green.channel, led_rgb->led_green.duty));
    // Set duty for blue LED
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_blue.channel, led_rgb->led_blue.duty));
    // Update duty to apply the new value for red LED
    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_red.channel));
    // Update duty to apply the new value for green LED
    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_green.channel));
    // Update duty to apply the new value for blue LED
    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_blue.channel));
}

void set_led_rgb_given_values(led_rgb_t *led_rgb, uint32_t duty_red, uint32_t duty_green, uint32_t duty_blue)
{
    led_rgb->led_red.duty = duty_red;
    led_rgb->led_green.duty = duty_green;
    led_rgb->led_blue.duty = duty_blue;
    // Set duty for red LED
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_red.channel, duty_red));
    // Set duty for green LED
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_green.channel, duty_green));
    // Set duty for blue LED
    ESP_ERROR_CHECK(ledc_set_duty(led_rgb->speed_mode, led_rgb->led_blue.channel, duty_blue));
    // Update duty to apply the new value for red LED
    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_red.channel));
    // Update duty to apply the new value for green LED
    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_green.channel));
    // Update duty to apply the new value for blue LED
    ESP_ERROR_CHECK(ledc_update_duty(led_rgb->speed_mode, led_rgb->led_blue.channel));
}

void set_led_rgb_percentage_given_values(led_rgb_t *led_rgb, int percentage_red, int percentage_green, int percentage_blue)
{
   /* uint32_t duty_red = pow(2, led_rgb->duty_resolution) * percentage_red / 100;
    uint32_t duty_green = pow(2, led_rgb->duty_resolution) * percentage_green / 100;
    uint32_t duty_blue = pow(2, led_rgb->duty_resolution) * percentage_blue / 100;
    set_led_rgb_given_values(led_rgb, duty_red, duty_green, duty_blue);

*/
    // Guarda los porcentajes actuales en la estructura      <-- CAMBIO
    led_rgb->led_red.percentage   = percentage_red;
    led_rgb->led_green.percentage = percentage_green;
    led_rgb->led_blue.percentage  = percentage_blue;
 
    uint32_t duty_red   = (uint32_t)(pow(2, led_rgb->duty_resolution) * percentage_red   / 100.0);
    uint32_t duty_green = (uint32_t)(pow(2, led_rgb->duty_resolution) * percentage_green / 100.0);
    uint32_t duty_blue  = (uint32_t)(pow(2, led_rgb->duty_resolution) * percentage_blue  / 100.0);
 
    set_led_rgb_given_values(led_rgb, duty_red, duty_green, duty_blue);
}
 
// NUEVA FUNCION ---------------------------------------------------------------
// Incrementa en 10% la intensidad del color indicado.
// color: 0 = rojo, 1 = verde, 2 = azul
// Al llegar a 100% vuelve a 0% (ciclo completo).
void increment_led_color(led_rgb_t *led_rgb, int color)
{
    int *pct = NULL;
 
    if      (color == 0) pct = &led_rgb->led_red.percentage;
    else if (color == 1) pct = &led_rgb->led_green.percentage;
    else if (color == 2) pct = &led_rgb->led_blue.percentage;
    else return;  // color inválido, no hace nada
 
    // Sube 10%; si supera 100 reinicia a 0
    *pct += 10;
    if (*pct > 100) *pct = 0;
 
    // Aplica los tres canales con el porcentaje actualizado
    set_led_rgb_percentage_given_values(
        led_rgb,
        led_rgb->led_red.percentage,
        led_rgb->led_green.percentage,
        led_rgb->led_blue.percentage
    );
 
    printf("R:%3d%%  G:%3d%%  B:%3d%%\n",
        led_rgb->led_red.percentage,
        led_rgb->led_green.percentage,
        led_rgb->led_blue.percentage);
}
 