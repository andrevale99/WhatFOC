#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "svpwm.h"
#include "inversor.h"

static const char *TAG = "SVPWM_TEST";

/* --------------------------------------------------------------------------
 * Inversor
 * -------------------------------------------------------------------------- */

// static const inversor_config_t inv_cfg = {
//     .U_H = GPIO_NUM_4,
//     .U_L = GPIO_NUM_5,
//     .V_H = GPIO_NUM_18,
//     .V_L = GPIO_NUM_19,
//     .W_H = GPIO_NUM_21,
//     .W_L = GPIO_NUM_22,
// };

static const inversor_config_t inv_cfg = {
    .U_H = GPIO_NUM_18,
    .U_L = GPIO_NUM_19,
    .V_H = GPIO_NUM_21,
    .V_L = GPIO_NUM_22,
    .W_H = GPIO_NUM_23,
    .W_L = GPIO_NUM_16,
};
/* --------------------------------------------------------------------------
 * Parâmetros da senoide gerada por software
 * -------------------------------------------------------------------------- */

#define SVPWM_MAGNITUDE 0.50f /* raio do vetor [0, 0.577]       */
#define SVPWM_FREQ_HZ 1.0f    /* frequência da fundamental (Hz)  */
#define LOOP_PERIOD_MS 10     /* período do loop em ms           */

/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_ERROR_CHECK(inversor_init(&inv_cfg));
    ESP_ERROR_CHECK(inversor_enable(true));

    ESP_LOGI(TAG, "Gerando senoide: %.1f Hz, magnitude %.2f", SVPWM_FREQ_HZ, SVPWM_MAGNITUDE);

    const float dt = LOOP_PERIOD_MS / 1000.0f;                  /* segundos por iteração   */
    const float delta_angle = 2.0f * M_PI * SVPWM_FREQ_HZ * dt; /* avanço de ângulo/iter   */

    float angle = 0.0f;

    while (1)
    {
        /* Vetor girante em coordenadas αβ */
        svpwm_ab_t ab = {
            .alpha = SVPWM_MAGNITUDE * cosf(angle),
            .beta = SVPWM_MAGNITUDE * sinf(angle),
        };

        svpwm_duty_t duty = {0};

        if (svpwm_calculate(&ab, &duty) == ESP_OK)
        {
            inversor_set_duty(duty.u, duty.v, duty.w);
        }

        // ESP_LOGI(TAG,
        //          "ang=%6.1f° α=%+.3f β=%+.3f | U=%.3f V=%.3f W=%.3f",
        //          angle * (180.0f / M_PI),
        //          ab.alpha, ab.beta,
        //          duty.u, duty.v, duty.w);

        /* Avança o ângulo e mantém em [0, 2π) */
        angle += delta_angle;
        if (angle >= 2.0f * M_PI)
            angle -= 2.0f * M_PI;

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}