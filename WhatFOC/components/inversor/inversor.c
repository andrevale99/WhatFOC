/**
 * @file inversor.c
 * @brief Driver de inversor trifásico usando MCPWM do ESP-IDF.
 */

#include "inversor.h"
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

/* -----------------------------------------------------------------------
 * Configurações do timer MCPWM
 * ----------------------------------------------------------------------- */

/** Frequência de chaveamento em Hz (ex.: 10 kHz) */
#define INVERSOR_PWM_FREQ_HZ 10000

/**
 * Resolução do timer: define a granularidade do duty cycle.
 * Período em ticks = TIMER_RESOLUTION_HZ / PWM_FREQ_HZ
 * Com 10 MHz e 10 kHz → 1000 ticks totais → resolução de 0,1%
 */
#define INVERSOR_TIMER_RESOLUTION_HZ 10000000UL /* 10 MHz */

/**
 * Período em ticks do timer (half-period para up-down).
 * O timer MCPWM em modo up-down conta até este valor e volta.
 */
#define INVERSOR_TIMER_PERIOD_TICKS \
    (INVERSOR_TIMER_RESOLUTION_HZ / INVERSOR_PWM_FREQ_HZ)

/* Valor máximo do comparador = period_ticks / 2 */
#define INVERSOR_COMPARATOR_MAX \
    (INVERSOR_TIMER_PERIOD_TICKS / 2)

/**
 * Dead-time em nanosegundos — tempo mínimo entre desligar um lado
 * e ligar o oposto do half-bridge para evitar shoot-through.
 */
#define INVERSOR_DEAD_TIME_NS 50U /* 150 ns */

/* -----------------------------------------------------------------------
 * Contexto interno (singleton)
 * ----------------------------------------------------------------------- */

static const char *TAG = "inversor";

/** Contexto MCPWM interno, alocado em inversor_init() */
typedef struct
{
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operators[3]; /* U=0, V=1, W=2 */
    mcpwm_cmpr_handle_t comparators[3];
    mcpwm_gen_handle_t generators[3][2]; /* [fase][0=Hi, 1=Lo] */
    bool initialized;
    bool running;
} inversor_ctx_t;

static inversor_ctx_t s_ctx = {0};

/* -----------------------------------------------------------------------
 * Helpers internos
 * ----------------------------------------------------------------------- */

/**
 * @brief Limita um float ao intervalo [lo, hi].
 */
static inline float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/**
 * @brief Converte duty cycle normalizado [0,1] em ticks do comparador.
 *
 * Aplica um teto seguro para evitar falhas críticas do validador
 * do driver MCPWM no modo UP_DOWN ao aproximar-se de 100%.
 */
static inline uint32_t duty_to_ticks(float duty)
{
    duty = clampf(duty, 0.0f, 1.0f);

    const uint32_t max_ticks = INVERSOR_COMPARATOR_MAX - 1;

    return (uint32_t)(duty * (float)max_ticks);
}
/* -------------------------- API pública ---------------------------------
 * ----------------------------------------------------------------------- */

esp_err_t inversor_init(const inversor_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "config é NULL");
    ESP_RETURN_ON_FALSE(!s_ctx.initialized, ESP_ERR_INVALID_STATE, TAG,
                        "inversor já inicializado");

    esp_err_t ret = ESP_OK;

    /* ------------------------------------------------------------------
     * 1. Timer MCPWM — compartilhado pelas 3 fases
     * Modo up-down (center-aligned) → PWM simétrico, ideal para SVPWM
     * ------------------------------------------------------------------ */
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = INVERSOR_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN, /* center-aligned */
        .period_ticks = INVERSOR_TIMER_PERIOD_TICKS,
    };
    ESP_GOTO_ON_ERROR(
        mcpwm_new_timer(&timer_cfg, &s_ctx.timer),
        err, TAG, "Falha ao criar timer MCPWM");

    ESP_LOGI(TAG,
             "PERIOD=%lu",
             (unsigned long)INVERSOR_TIMER_PERIOD_TICKS);

    ESP_LOGI(TAG,
             "PERIOD=%lu",
             (unsigned long)INVERSOR_TIMER_PERIOD_TICKS);

    /* ------------------------------------------------------------------
     * 2. Operadores — um por fase, todos conectados ao mesmo timer
     * ------------------------------------------------------------------ */
    mcpwm_operator_config_t oper_cfg = {
        .group_id = 0,
    };
    for (int i = 0; i < 3; i++)
    {
        ESP_GOTO_ON_ERROR(
            mcpwm_new_operator(&oper_cfg, &s_ctx.operators[i]),
            err, TAG, "Falha ao criar operador MCPWM [%d]", i);
        ESP_GOTO_ON_ERROR(
            mcpwm_operator_connect_timer(s_ctx.operators[i], s_ctx.timer),
            err, TAG, "Falha ao conectar operador [%d] ao timer", i);
    }

    /* ------------------------------------------------------------------
     * 3. Comparadores — controlam o duty cycle de cada fase
     * ------------------------------------------------------------------ */
    mcpwm_comparator_config_t cmp_cfg = {
        .flags.update_cmp_on_tez = true,
    };
    for (int i = 0; i < 3; i++)
    {
        ESP_GOTO_ON_ERROR(
            mcpwm_new_comparator(s_ctx.operators[i], &cmp_cfg, &s_ctx.comparators[i]),
            err, TAG, "Falha ao criar comparador [%d]", i);
        /* Inicia com duty zero (saída desligada) */
        ESP_GOTO_ON_ERROR(
            mcpwm_comparator_set_compare_value(s_ctx.comparators[i], 0),
            err, TAG, "Falha ao inicializar comparador [%d]", i);
    }

    /* ------------------------------------------------------------------
     * 4. Geradores — Hi-side e Lo-side para cada fase
     * ------------------------------------------------------------------ */
    const gpio_num_t gpio_map[3][2] = {
        {config->U_H, config->U_L},
        {config->V_H, config->V_L},
        {config->W_H, config->W_L},
    };

    mcpwm_generator_config_t gen_cfg = {};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            gen_cfg.gen_gpio_num = gpio_map[i][j];
            ESP_GOTO_ON_ERROR(
                mcpwm_new_generator(s_ctx.operators[i], &gen_cfg, &s_ctx.generators[i][j]),
                err, TAG, "Falha ao criar gerador fase=%d side=%s GPIO=%d",
                i, j == 0 ? "Hi" : "Lo", gpio_map[i][j]);
        }
    }

    /* ------------------------------------------------------------------
     * 5. Ações dos geradores Hi-side
     * ------------------------------------------------------------------ */
    for (int i = 0; i < 3; i++)
    {
        ESP_GOTO_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(
                s_ctx.generators[i][0],
                MCPWM_GEN_COMPARE_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP,
                    s_ctx.comparators[i],
                    MCPWM_GEN_ACTION_LOW)),
            err, TAG, "Falha ao configurar ação UP fase [%d]", i);
        ESP_GOTO_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(
                s_ctx.generators[i][0],
                MCPWM_GEN_COMPARE_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_DOWN,
                    s_ctx.comparators[i],
                    MCPWM_GEN_ACTION_HIGH)),
            err, TAG, "Falha ao configurar ação DOWN fase [%d]", i);
    }

    /* ------------------------------------------------------------------
     * 6. Dead-time
     * ------------------------------------------------------------------ */
    mcpwm_dead_time_config_t dt_hi = {
        .posedge_delay_ticks = (uint32_t)((float)INVERSOR_DEAD_TIME_NS * 1e-9f * INVERSOR_TIMER_RESOLUTION_HZ),
    };

    mcpwm_dead_time_config_t dt_lo = {
        .negedge_delay_ticks = dt_hi.posedge_delay_ticks,
        .flags.invert_output = true,
    };

    for (int i = 0; i < 3; i++)
    {
        ESP_GOTO_ON_ERROR(
            mcpwm_generator_set_dead_time(
                s_ctx.generators[i][0], s_ctx.generators[i][0], &dt_hi),
            err, TAG, "Falha ao configurar dead-time Hi fase [%d]", i);
        ESP_GOTO_ON_ERROR(
            mcpwm_generator_set_dead_time(
                s_ctx.generators[i][0], s_ctx.generators[i][1], &dt_lo),
            err, TAG, "Falha ao configurar dead-time Lo fase [%d]", i);
    }

    s_ctx.initialized = true;
    ESP_LOGI(TAG, "Inversor inicializado: f=%u Hz, resolução=%lu ticks, dead-time=%u ns",
             (unsigned)INVERSOR_PWM_FREQ_HZ,
             (unsigned long)INVERSOR_TIMER_PERIOD_TICKS,
             (unsigned)INVERSOR_DEAD_TIME_NS);
    return ESP_OK;

err:
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            if (s_ctx.generators[i][j])
                mcpwm_del_generator(s_ctx.generators[i][j]);
        }
        if (s_ctx.comparators[i])
            mcpwm_del_comparator(s_ctx.comparators[i]);
        if (s_ctx.operators[i])
            mcpwm_del_operator(s_ctx.operators[i]);
    }
    if (s_ctx.timer)
        mcpwm_del_timer(s_ctx.timer);
    memset(&s_ctx, 0, sizeof(s_ctx));
    return ret;
}

esp_err_t inversor_register_callback(const mcpwm_timer_event_callbacks_t *cbs, void *user_ctx)
{
    ESP_RETURN_ON_FALSE(s_ctx.initialized, ESP_ERR_INVALID_STATE, TAG, "Inversor não inicializado");
    return mcpwm_timer_register_event_callbacks(s_ctx.timer, cbs, user_ctx);
}

esp_err_t inversor_set_duty(float duty_u, float duty_v, float duty_w)
{
    ESP_RETURN_ON_FALSE(s_ctx.initialized, ESP_ERR_INVALID_STATE, TAG,
                        "inversor não inicializado");

    const uint32_t ticks_u = duty_to_ticks(duty_u);
    const uint32_t ticks_v = duty_to_ticks(duty_v);
    const uint32_t ticks_w = duty_to_ticks(duty_w);

    ESP_RETURN_ON_ERROR(
        mcpwm_comparator_set_compare_value(s_ctx.comparators[0], ticks_u),
        TAG, "Falha ao setar duty U");
    ESP_RETURN_ON_ERROR(
        mcpwm_comparator_set_compare_value(s_ctx.comparators[1], ticks_v),
        TAG, "Falha ao setar duty V");
    ESP_RETURN_ON_ERROR(
        mcpwm_comparator_set_compare_value(s_ctx.comparators[2], ticks_w),
        TAG, "Falha ao setar duty W");

    // ESP_LOGI(TAG, "Duty cycles atualizados: U=%.3f V=%.3f W=%.3f", duty_u, duty_v, duty_w);

    return ESP_OK;
}

esp_err_t inversor_enable(bool enable)
{
    ESP_RETURN_ON_FALSE(s_ctx.initialized, ESP_ERR_INVALID_STATE, TAG,
                        "inversor não inicializado");

    if (enable && !s_ctx.running)
    {
        ESP_RETURN_ON_ERROR(
            mcpwm_timer_enable(s_ctx.timer),
            TAG, "Falha ao habilitar timer");
        ESP_RETURN_ON_ERROR(
            mcpwm_timer_start_stop(s_ctx.timer, MCPWM_TIMER_START_NO_STOP),
            TAG, "Falha ao iniciar timer");
        s_ctx.running = true;
        ESP_LOGI(TAG, "Inversor habilitado");
    }
    else if (!enable && s_ctx.running)
    {
        ESP_RETURN_ON_ERROR(
            mcpwm_timer_start_stop(s_ctx.timer, MCPWM_TIMER_STOP_EMPTY),
            TAG, "Falha ao parar timer");
        ESP_RETURN_ON_ERROR(
            mcpwm_timer_disable(s_ctx.timer),
            TAG, "Falha ao desabilitar timer");
        s_ctx.running = false;
        ESP_LOGI(TAG, "Inversor desabilitado");
    }

    return ESP_OK;
}