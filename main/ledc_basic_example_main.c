/* LEDC (LED Controller) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "library_led_c.h"
/*
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz
#define LED_RGB_1_RED_GPIO       (7)
#define LED_RGB_1_GREEN_GPIO     (18)
#define LED_RGB_1_BLUE_GPIO      (19)



#define BUTTON_RED     2
#define BUTTON_GREEN   3
#define BUTTON_BLUE    4

#define DUTY_STEP      512   // paso de incremento de intensidad
#define DUTY_MAX       8191  // máximo para 13 bits


static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
*/
// ── Pines del LED RGB (cátodo común) ─────────────────────────────────────────
#define LED_RGB_1_RED_GPIO    (7)
#define LED_RGB_1_GREEN_GPIO  (18)
#define LED_RGB_1_BLUE_GPIO   (19)
 
// ── Pines de los botones ──────────────────────────────────────────────────────
// Cada botón sube 10% la intensidad de su color
#define BUTTON_RED    2
#define BUTTON_GREEN  3
#define BUTTON_BLUE   4
 
// ── Configura los tres GPIOs de botón como entradas con pull-up interno ───────
// Al presionar el botón el pin lee 0 (activo en bajo).
static void buttons_init(void)
{
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_RED) |
                        (1ULL << BUTTON_GREEN) |
                        (1ULL << BUTTON_BLUE),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,    // pull-up interno activado
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_cfg);
}
void app_main(void)
{
    led_rgb_t led_rgb1 = {
        .led_red = {
            .duty = 0,
            .gpio_num = LED_RGB_1_RED_GPIO,
            .channel = LEDC_CHANNEL_0,
            .percentage = 0          // <-- inicia en 0%
        },
        .led_green = {
            .duty = 0,
            .gpio_num = LED_RGB_1_GREEN_GPIO,
            .channel = LEDC_CHANNEL_1,
            .percentage = 0          // <-- inicia en 0%
        },
        .led_blue = {
            .duty = 0,
            .gpio_num = LED_RGB_1_BLUE_GPIO,
            .channel = LEDC_CHANNEL_2,
            .percentage = 0          // <-- inicia en 0%
        },
        .timer = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .frequency = 4000,
        .speed_mode = LEDC_LOW_SPEED_MODE
    };
    config_led_rgb(&led_rgb1);


    // ── 2. Inicializar botones ────────────────────────────────────────────────
    buttons_init();
 
    // ── 3. Variables para detectar flanco de bajada (evita rebotes simples) ──
    // Se considera "presionado" cuando el pin pasa de 1 → 0.
    int prev_red   = 1;
    int prev_green = 1;
    int prev_blue  = 1;
 
    printf("Sistema listo. Presiona los botones para subir intensidad.\n");
 
    // ── 4. Loop principal ─────────────────────────────────────────────────────
    while (1) {
 
        int cur_red   = gpio_get_level(BUTTON_RED);
        int cur_green = gpio_get_level(BUTTON_GREEN);
        int cur_blue  = gpio_get_level(BUTTON_BLUE);
 
        // Botón ROJO presionado (flanco bajada: 1 → 0)
        if (prev_red == 1 && cur_red == 0) {
            increment_led_color(&led_rgb1, 0);   // 0 = rojo
        }
 
        // Botón VERDE presionado
        if (prev_green == 1 && cur_green == 0) {
            increment_led_color(&led_rgb1, 1);   // 1 = verde
        }
 
        // Botón AZUL presionado
        if (prev_blue == 1 && cur_blue == 0) {
            increment_led_color(&led_rgb1, 2);   // 2 = azul
        }
 
        // Guarda estado actual para detectar el próximo flanco
        prev_red   = cur_red;
        prev_green = cur_green;
        prev_blue  = cur_blue;
 
        vTaskDelay(20 / portTICK_PERIOD_MS);  // 20 ms — anti-rebote básico
    }
}
/*
    while(1) {
        set_led_rgb_percentage_given_values(&led_rgb1, 50, 50, 50);
        //Vtask_delay(1000 / portTICK_PERIOD_MS);
        set_led_rgb_percentage_given_values(&led_rgb1, 100, 0, 0);
        //Vtask_delay(1000 / portTICK_PERIOD_MS);

    }
        

    // Set the LEDC peripheral configuration
    //example_ledc_init();
    // Set duty to 50%
    //ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    //ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}*/
