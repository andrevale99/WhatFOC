#ifndef INVERSOR_H
#define INVERSOR_H

/*
 */

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

typedef enum
{
    INVERSOR_PHASE_U = 0,
    INVERSOR_PHASE_V = 1,
    INVERSOR_PHASE_W = 2,
} inversor_phase_t;

typedef struct inversor_config_t
{
    gpio_num_t U_H; /* GPIO para fase U alta  */
    gpio_num_t U_L; /* GPIO para fase U baixa */
    gpio_num_t V_H; /* GPIO para fase V alta  */
    gpio_num_t V_L; /* GPIO para fase V baixa */
    gpio_num_t W_H; /* GPIO para fase W alta  */
    gpio_num_t W_L; /* GPIO para fase W baixa */
} inversor_config_t;

/**
 * @brief Registra callbacks de eventos para o timer do inversor
 */
esp_err_t inversor_register_callback(const mcpwm_timer_event_callbacks_t *cbs, void *user_ctx);

esp_err_t inversor_init(const inversor_config_t *config);
esp_err_t inversor_set_duty(float duty_u, float duty_v, float duty_w);
esp_err_t inversor_enable(bool enable);

#endif