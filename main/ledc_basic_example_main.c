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
 
 /*
 * Proyecto: LED RGB dual + UART
 *   LED 1 → controlado por termistor NTC con rangos configurables por UART
 *   LED 2 → máquina de estados con 1 botón + 1 potenciómetro
 *
 * Comandos UART (escribir en monitor serie + Enter):
 *   Grupo 1 - Límites de temperatura por color (LED1):
 *     LIM_MI_R_<val>   mínimo de temperatura para rojo   (ej: LIM_MI_R_5)
 *     LIM_MA_R_<val>   máximo de temperatura para rojo   (ej: LIM_MA_R_20)
 *     LIM_MI_B_<val>   mínimo de temperatura para azul   (ej: LIM_MI_B_10)
 *     LIM_MA_B_<val>   máximo de temperatura para azul   (ej: LIM_MA_B_30)
 *     LIM_MI_G_<val>   mínimo de temperatura para verde  (ej: LIM_MI_G_15)
 *     LIM_MA_G_<val>   máximo de temperatura para verde  (ej: LIM_MA_G_40)
 *   Grupo 2 - Intensidad fija por color (ambos LEDs):
 *     INT_R_<val>      intensidad rojo   0-100  (ej: INT_R_80)
 *     INT_B_<val>      intensidad azul   0-100  (ej: INT_B_50)
 *     INT_G_<val>      intensidad verde  0-100  (ej: INT_G_30)
 *   Grupo 3 - Lectura:
 *     READ             imprime todos los valores actuales
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
#include "driver/uart.h"
#include <string.h>
#include "library_led_c.h"




static const char *TAG = "LED_RGB";

// ── Variables globales configurables por UART ─────────────────────────────────
// Límites de temperatura por color (LED1)
static int lim_min_red   = 5,  lim_max_red   = 15;
static int lim_min_blue  = 16, lim_max_blue  = 25;
static int lim_min_green = 26, lim_max_green = 40;

// Intensidad fija por color (ambos LEDs)
static int int_red   = 100;
static int int_blue  = 100;
static int int_green = 100;

// Última temperatura leída (para el comando READ)
static float last_temp     = 0.0f;
static int   last_volt_mv  = 0;
static float last_r_therm  = 0.0f;
static int   last_pot_pct  = 0;
 
// ── GPIOs LED 1 (termistor) ───────────────────────────────────────────────────
#define LED1_RED_GPIO    13
#define LED1_GREEN_GPIO  12
#define LED1_BLUE_GPIO   11
 
// ── GPIOs LED 2 (estados) ─────────────────────────────────────────────────────
#define LED2_RED_GPIO    10
#define LED2_GREEN_GPIO  8 
#define LED2_BLUE_GPIO   7
 
// ── ADC ───────────────────────────────────────────────────────────────────────
#define ADC_THERMISTOR   ADC_CHANNEL_4   // GPIO 4  → termistor
#define ADC_POT          ADC_CHANNEL_5   // GPIO 5  → potenciómetro
#define ADC_ATTEN        ADC_ATTEN_DB_12 // rango 0-3.3V
 
// ── Botón ─────────────────────────────────────────────────────────────────────
#define BUTTON_GPIO      6               // mismo botón que antes
 
// ── Termistor NTCLE100E3101JB0A ───────────────────────────────────────────────
// R0 = 10000 Ω a T0 = 25°C, B = 4500 K
// Divisor de voltaje: 3.3V → R_fija(1000Ω) → pin ADC → termistor → GND
#define THERMISTOR_R0    6957.0f
#define THERMISTOR_B     4500.0f
#define THERMISTOR_T0    298.15f   // 25°C en Kelvin
#define THERMISTOR_RFIJA 13000.0f   // resistencia fija del divisor (1000 Ω)
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
// out_voltage_mv: voltaje medido en mV (parámetro de salida)  <-- NUEVO
// ─────────────────────────────────────────────────────────────────────────────
static float adc_to_temperature(int raw, int *out_voltage_mv)  // <-- NUEVO: agrega out_voltage_mv
{
    // Voltaje leído
    float v_adc = (raw / ADC_MAX) * 3.3f;

    // NUEVO: guarda el voltaje en mV en el puntero recibido
    if (out_voltage_mv != NULL) {
        *out_voltage_mv = (int)(v_adc * 1000.0f);
    }

    // Resistencia del termistor en el divisor: Rt = Rfija * Vadc / (3.3 - Vadc)
    if (v_adc >= 3.3f) v_adc = 3.29f; // evitar división por cero
    float r_thermistor = THERMISTOR_RFIJA * v_adc / (3.3f - v_adc);
   // float r_thermistor = THERMISTOR_RFIJA * (3.3f - v_adc) / v_adc;
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
// ── UART ─────────────────────────────────────────────────────────────────────
#define UART_PORT       UART_NUM_0
#define UART_BUF_SIZE   256

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    // GPIO 0 y 1 son TX/RX del UART0 por defecto en ESP32-C6
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, 16, 17, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void uart_task(void *arg)
{
    uint8_t buf[UART_BUF_SIZE];
    int     val;

    while (1) {
        int len = uart_read_bytes(UART_PORT, buf, UART_BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len <= 0) continue;

        buf[len] = '\0';
        // Elimina \r y \n del final
        for (int i = len - 1; i >= 0 && (buf[i] == '\r' || buf[i] == '\n'); i--) buf[i] = '\0';

        ESP_LOGI(TAG, "CMD recibido: %s", (char*)buf);

        // ── Grupo 1: límites de temperatura ──────────────────────────────────
        if      (sscanf((char*)buf, "LIM_MI_R_%d", &val) == 1) { lim_min_red   = val; uart_write_bytes(UART_PORT, "OK: LIM_MI_R\r\n", 14); }
        else if (sscanf((char*)buf, "LIM_MA_R_%d", &val) == 1) { lim_max_red   = val; uart_write_bytes(UART_PORT, "OK: LIM_MA_R\r\n", 14); }
        else if (sscanf((char*)buf, "LIM_MI_B_%d", &val) == 1) { lim_min_blue  = val; uart_write_bytes(UART_PORT, "OK: LIM_MI_B\r\n", 14); }
        else if (sscanf((char*)buf, "LIM_MA_B_%d", &val) == 1) { lim_max_blue  = val; uart_write_bytes(UART_PORT, "OK: LIM_MA_B\r\n", 14); }
        else if (sscanf((char*)buf, "LIM_MI_G_%d", &val) == 1) { lim_min_green = val; uart_write_bytes(UART_PORT, "OK: LIM_MI_G\r\n", 14); }
        else if (sscanf((char*)buf, "LIM_MA_G_%d", &val) == 1) { lim_max_green = val; uart_write_bytes(UART_PORT, "OK: LIM_MA_G\r\n", 14); }

        // ── Grupo 2: intensidad por color ─────────────────────────────────────
        else if (sscanf((char*)buf, "INT_R_%d", &val) == 1) { int_red   = val; uart_write_bytes(UART_PORT, "OK: INT_R\r\n", 11); }
        else if (sscanf((char*)buf, "INT_B_%d", &val) == 1) { int_blue  = val; uart_write_bytes(UART_PORT, "OK: INT_B\r\n", 11); }
        else if (sscanf((char*)buf, "INT_G_%d", &val) == 1) { int_green = val; uart_write_bytes(UART_PORT, "OK: INT_G\r\n", 11); }

        // ── Grupo 3: READ ─────────────────────────────────────────────────────
        else if (strcmp((char*)buf, "READ") == 0) {
            char resp[300];
            snprintf(resp, sizeof(resp),
                "\r\n===== ESTADO ACTUAL =====\r\n"
                "Temp      : %.1f C\r\n"
                "Voltaje   : %d mV\r\n"
                "Resist.   : %.1f Ohm\r\n"
                "Pot       : %d%%\r\n"
                "LIM Rojo  : %d - %d C  | INT: %d%%\r\n"
                "LIM Verde : %d - %d C  | INT: %d%%\r\n"
                "LIM Azul  : %d - %d C  | INT: %d%%\r\n"
                "=========================\r\n",
                last_temp, last_volt_mv, last_r_therm, last_pot_pct,
                lim_min_red,   lim_max_red,   int_red,
                lim_min_green, lim_max_green, int_green,
                lim_min_blue,  lim_max_blue,  int_blue
            );
            uart_write_bytes(UART_PORT, resp, strlen(resp));
        }
        else {
            uart_write_bytes(UART_PORT, "ERROR: comando desconocido\r\n", 28);
        }
    }
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
    uart_init();                                                          // <-- NUEVO
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);            // <-- NUEVO


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
        int voltage_mv = 0;                                           // <-- NUEVO
        float temp = adc_to_temperature(raw_therm, &voltage_mv);     // <-- NUEVO: pasa &voltage_mv
 


      // calcula la resistencia del termistor desde el voltaje medido
        // Fórmula del divisor despejada: Rt = Rfija * Vadc / (3.3 - Vadc)
        float v_adc = voltage_mv / 1000.0f;
        float r_therm = THERMISTOR_RFIJA * v_adc / (3.3f - v_adc);
    /*
        if (temp < 25.0f) {
            led_rgb_set_color(&led1, 0, 0, 100);   // azul
        } else if (temp <= 35.0f) {
            led_rgb_set_color(&led1, 0, 100, 0);   // verde
        } else {
            led_rgb_set_color(&led1, 100, 0, 0);   // rojo
            }*/
       
            // Cada color se evalúa independientemente según sus límites configurables
        int r = (temp >= lim_min_red   && temp <= lim_max_red)   ? int_red   : 0;
        int g = (temp >= lim_min_green && temp <= lim_max_green) ? int_green : 0;
        int b = (temp >= lim_min_blue  && temp <= lim_max_blue)  ? int_blue  : 0;
        led_rgb_set_color(&led1, r, g, b);
        ESP_LOGI(TAG, "Temp: %.1f C | Voltaje: %d mV | Resistencia: %.1f Ohm", temp, voltage_mv, r_therm);  // <-- NUEVO: agrega voltaje
        
        // Guarda los valores para que el comando READ pueda reportarlos
        last_temp    = temp;
        last_volt_mv = voltage_mv;
        last_r_therm = r_therm;
       
       
        // ── Leer potenciómetro (0-4095 → 0-100%) ─────────────────────────────
        int raw_pot = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_POT, &raw_pot));
        int pot_pct = (int)((raw_pot / ADC_MAX) * 100.0f);  // 0 a 100
        last_pot_pct = pot_pct;   // guarda para READ
        
        
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
 
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 20 ms — lectura ADC + anti-rebote
    }
}