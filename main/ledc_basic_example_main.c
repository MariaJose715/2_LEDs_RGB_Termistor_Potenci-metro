/*
 * Proyecto: LED RGB dual
 *   LED 1 → controlado por termistor NTC (NTCLE100E3101JB0A, R0=100Ω, B=4250K)
 *             < 25°C  → azul
 *             25-35°C → verde
 *             > 35°C  → rojo
 *
 *   LED 2 → máquina de estados con 1 botón + 1 potenciómetro
 *             Estado 0 (ROJO)  : varías R con el potenciómetro, botón guarda y pasa al estado 1
 *             Estado 1 (AZUL)  : varías B con el potenciómetro, botón guarda y pasa al estado 2
 *             Estado 2 (VERDE) : varías G con el potenciómetro, botón guarda y pasa al estado 3
 *             Estado 3 (SHOW)  : muestra los 3 colores guardados. Botón reinicia desde estado 0.
 */
 
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"
#include "esp_log.h"
#include "library_led_c.h"


static const char *TAG = "LED_RGB";
 
// ── GPIOs LED 1 (termistor) ───────────────────────────────────────────────────
#define LED1_RED_GPIO    7
#define LED1_GREEN_GPIO  18
#define LED1_BLUE_GPIO   19
 
// ── GPIOs LED 2 (estados) ─────────────────────────────────────────────────────
#define LED2_RED_GPIO    8
#define LED2_GREEN_GPIO  9
#define LED2_BLUE_GPIO   10
 
// ── ADC ───────────────────────────────────────────────────────────────────────
#define ADC_THERMISTOR   ADC_CHANNEL_4   // GPIO 4  → termistor
#define ADC_POT          ADC_CHANNEL_5   // GPIO 5  → potenciómetro
#define ADC_ATTEN        ADC_ATTEN_DB_12 // rango 0-3.3V
 
// ── Botón ─────────────────────────────────────────────────────────────────────
#define BUTTON_GPIO      2               // mismo botón que antes
 
// ── Termistor NTCLE100E3101JB0A ───────────────────────────────────────────────
// R0 = 100 Ω a T0 = 25°C, B = 4250 K
// Divisor de voltaje: 3.3V → R_fija(100Ω) → pin ADC → termistor → GND
#define THERMISTOR_R0    100.0f
#define THERMISTOR_B     4250.0f
#define THERMISTOR_T0    298.15f   // 25°C en Kelvin
#define THERMISTOR_RFIJA 100.0f    // resistencia fija del divisor (100 Ω)
#define ADC_MAX          4095.0f   // resolución 12 bits
 
// ── Estados del LED 2 ─────────────────────────────────────────────────────────
typedef enum {
    STATE_RED   = 0,   // ajustando rojo
    STATE_BLUE  = 1,   // ajustando azul
    STATE_GREEN = 2,   // ajustando verde
    STATE_SHOW  = 3    // mostrando resultado
} led2_state_t;
 
// ─────────────────────────────────────────────────────────────────────────────
// Convierte lectura ADC del termistor a temperatura en °C
// Fórmula Beta: 1/T = 1/T0 + (1/B)*ln(R/R0)
// ─────────────────────────────────────────────────────────────────────────────
static float adc_to_temperature(int raw)
{
    // Voltaje leído
    float v_adc = (raw / ADC_MAX) * 3.3f;
 
    // Resistencia del termistor en el divisor: Rt = Rfija * Vadc / (3.3 - Vadc)
    if (v_adc >= 3.3f) v_adc = 3.29f; // evitar división por cero
    float r_thermistor = THERMISTOR_RFIJA * v_adc / (3.3f - v_adc);
 
    // Ecuación Beta
    float inv_T = (1.0f / THERMISTOR_T0) + (1.0f / THERMISTOR_B) * logf(r_thermistor / THERMISTOR_R0);
    float temp_k = 1.0f / inv_T;
    return temp_k - 273.15f; // convertir a Celsius
}
 
// ─────────────────────────────────────────────────────────────────────────────
// Inicializa ADC oneshot para dos canales (termistor y potenciómetro)
// Tomado del ejemplo oneshot_read_main de Espressif, simplificado
// ─────────────────────────────────────────────────────────────────────────────
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t         adc1_cali_therm = NULL;
static adc_cali_handle_t         adc1_cali_pot   = NULL;
 
static void adc_init(void)
{
    // Init unidad ADC1
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));
 
    // Configuración común para ambos canales
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT   // 12 bits en ESP32-C6
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_THERMISTOR, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_POT,        &chan_cfg));
 
    // Calibración (Curve Fitting en ESP32-C6)
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    cali_cfg.chan = ADC_THERMISTOR;
    adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc1_cali_therm);
    cali_cfg.chan = ADC_POT;
    adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc1_cali_pot);
#endif
}
 
// ─────────────────────────────────────────────────────────────────────────────
// Inicializa el botón con pull-up interno
// ─────────────────────────────────────────────────────────────────────────────
// ── Configura los tres GPIOs de botón como entradas con pull-up interno ───────
// Al presionar el botón el pin lee 0 (activo en bajo).
static void buttons_init(void)
{
    gpio_config_t cfg = {
       .pin_bit_mask = (1ULL << BUTTON_GPIO),
     //  .pin_bit_mask = (1ULL << BUTTON_RED) |
//                      (1ULL << BUTTON_GREEN) |
  //                      (1ULL << BUTTON_BLUE),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,    // pull-up interno activado
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
}
void app_main(void)
{
    led_rgb_t led1 = {
        .led_red = {
            .duty = 0,
            .gpio_num = LED1_RED_GPIO,
            .channel = LEDC_CHANNEL_0,
            .percentage = 0          // <-- inicia en 0%
        },
        .led_green = {
            .duty = 0,
            .gpio_num = LED1_GREEN_GPIO,
            .channel = LEDC_CHANNEL_1,
            .percentage = 0          // <-- inicia en 0%
        },
        .led_blue = {
            .duty = 0,
            .gpio_num = LED1_BLUE_GPIO,
            .channel = LEDC_CHANNEL_2,
            .percentage = 0          // <-- inicia en 0%
        },
        .timer = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .frequency = 4000,
        .speed_mode = LEDC_LOW_SPEED_MODE
    };
    config_led_rgb(&led1);


    // ── 2. Configurar LED 2 (estados) ────────────────────────────────────────
    // Usa TIMER_1 para no interferir con LED 1
    led_rgb_t led2 = {
        .led_red   = { .duty = 0, .gpio_num = LED2_RED_GPIO,   .channel = LEDC_CHANNEL_3, .percentage = 0 },
        .led_green = { .duty = 0, .gpio_num = LED2_GREEN_GPIO, .channel = LEDC_CHANNEL_4, .percentage = 0 },
        .led_blue  = { .duty = 0, .gpio_num = LED2_BLUE_GPIO,  .channel = LEDC_CHANNEL_5, .percentage = 0 },
        .timer           = LEDC_TIMER_1,       // <-- timer diferente al LED 1
        .duty_resolution = LEDC_TIMER_13_BIT,
        .frequency       = 4000,
        .speed_mode      = LEDC_LOW_SPEED_MODE
    };
    config_led_rgb(&led2);
 
    // ── 3. Inicializar ADC y botón ────────────────────────────────────────────
    adc_init();
    buttons_init();
 
    // ── 4. Variables de estado para LED 2 ────────────────────────────────────
    led2_state_t estado      = STATE_RED;  // empieza ajustando rojo
    int saved_red            = 0;
    int saved_green          = 0;
    int saved_blue           = 0;
    int prev_button          = 1;          // para detectar flanco
 
    // Enciende LED2 en rojo para indicar que empieza el estado 0
    led_rgb_set_color(&led2, 100, 0, 0);
 
    printf("Sistema listo.\n");
    printf("LED1 = termistor | LED2 = estados (boton GPIO%d, pot GPIO5)\n", BUTTON_GPIO);
 
    // ── 5. Loop principal ─────────────────────────────────────────────────────

       while (1) {
 
        // ── Leer termistor y actualizar LED 1 ────────────────────────────────
        int raw_therm = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_THERMISTOR, &raw_therm));
        float temp = adc_to_temperature(raw_therm);
 
        if (temp < 25.0f) {
            led_rgb_set_color(&led1, 0, 0, 100);   // azul
        } else if (temp <= 35.0f) {
            led_rgb_set_color(&led1, 0, 100, 0);   // verde
        } else {
            led_rgb_set_color(&led1, 100, 0, 0);   // rojo
        }
        ESP_LOGI(TAG, "Temp: %.1f°C", temp);
 
        // ── Leer potenciómetro (0-4095 → 0-100%) ─────────────────────────────
        int raw_pot = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_POT, &raw_pot));
        int pot_pct = (int)((raw_pot / ADC_MAX) * 100.0f);  // 0 a 100
 
        // ── Actualizar LED 2 según estado actual ──────────────────────────────
        switch (estado) {
            case STATE_RED:
                led_rgb_set_color(&led2, pot_pct, 0, 0);   // rojo varía con el pot
                break;
            case STATE_BLUE:
                led_rgb_set_color(&led2, 0, 0, pot_pct);   // azul varía con el pot
                break;
            case STATE_GREEN:
                led_rgb_set_color(&led2, 0, pot_pct, 0);   // verde varía con el pot
                break;
            case STATE_SHOW:
                // Muestra los tres colores guardados simultáneamente
                led_rgb_set_color(&led2, saved_red, saved_green, saved_blue);
                break;
        }
 
        // ── Detectar pulsación del botón (flanco 1 → 0) ───────────────────────
        int cur_button = gpio_get_level(BUTTON_GPIO);
        if (prev_button == 1 && cur_button == 0) {
 
            switch (estado) {
                case STATE_RED:
                    saved_red = pot_pct;               // guarda rojo
                    ESP_LOGI(TAG, "Rojo guardado: %d%%", saved_red);
                    led_rgb_off(&led2);
                    estado = STATE_BLUE;               // pasa a ajustar azul
                    break;
 
                case STATE_BLUE:
                    saved_blue = pot_pct;              // guarda azul
                    ESP_LOGI(TAG, "Azul guardado: %d%%", saved_blue);
                    led_rgb_off(&led2);
                    estado = STATE_GREEN;              // pasa a ajustar verde
                    break;
 
                case STATE_GREEN:
                    saved_green = pot_pct;             // guarda verde
                    ESP_LOGI(TAG, "Verde guardado: %d%%", saved_green);
                    estado = STATE_SHOW;               // pasa a mostrar resultado
                    ESP_LOGI(TAG, "SHOW → R:%d%% G:%d%% B:%d%%",
                             saved_red, saved_green, saved_blue);
                    break;
 
                case STATE_SHOW:
                    // Reinicia todo desde cero
                    saved_red = saved_green = saved_blue = 0;
                    estado = STATE_RED;
                    led_rgb_set_color(&led2, 100, 0, 0);  // enciende rojo para indicar inicio
                    ESP_LOGI(TAG, "Reiniciando estados...");
                    break;
            }
        }
        prev_button = cur_button;
 
        vTaskDelay(20 / portTICK_PERIOD_MS);  // 20 ms — lectura ADC + anti-rebote
    }
}

