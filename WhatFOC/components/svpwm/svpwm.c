/**
 * @file svpwm.c
 * @brief Space Vector PWM (SVPWM) - Cálculo de duty cycles para inversor trifásico.
 *
 * Recebe um vetor de tensão em coordenadas αβ (saída da transformada de Clarke)
 * e retorna os três duty cycles normalizados [0,1] para as fases U, V e W.
 *
 * O algoritmo é baseado na detecção do setor no plano αβ e no cálculo dos tempos
 * dos vetores ativos correspondentes, garantindo a modulação ideal para controle de motores.
 *
 * @author José Igo S. Camilo
 * @date 2026-06-22
 * @license MIT
 */

#include "svpwm.h"
#include <math.h>

#define SQRT3 1.73205080756887729353f      /* √3                  */
#define SQRT3_INV 0.57735026918962576451f  /* 1/√3                */
#define SQRT3_HALF 0.86602540378443864676f /* √3/2                */

/**
 * @brief Limita um valor float ao intervalo [min, max].
 * Inline para não gerar overhead de chamada em loop de controle.
 */
static inline float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

esp_err_t svpwm_calculate(const svpwm_ab_t *ab, svpwm_duty_t *duty)
{
    if (ab == NULL || duty == NULL)
        return ESP_ERR_INVALID_ARG;

    const float alpha = ab->alpha;
    const float beta = ab->beta;

    const float v_reg1 = beta;
    const float v_reg2 = (SQRT3 * alpha - beta) * 0.5f;
    const float v_reg3 = (-SQRT3 * alpha - beta) * 0.5f;

    /*
     * Tabela correta de sinais:
     *  Setor | v_reg1 | v_reg2 | v_reg3 | Faixa
     *    1   |   +    |   +    |   −    |   0–60°
     *    2   |   +    |   −    |   −    |  60–120°
     *    3   |   +    |   −    |   +    | 120–180°
     *    4   |   −    |   −    |   +    | 180–240°
     *    5   |   −    |   +    |   +    | 240–300°
     *    6   |   −    |   +    |   −    | 300–360°
     */
    int sector;

    if (v_reg1 >= 0.0f)
    {
        if (v_reg2 >= 0.0f)
            sector = 1; /* +, +, − */
        else if (v_reg3 >= 0.0f)
            sector = 3; /* +, −, + */
        else
            sector = 2; /* +, −, − */
    }
    else
    {
        if (v_reg2 < 0.0f)
            sector = 4; /* −, −, + */
        else if (v_reg3 < 0.0f)
            sector = 6; /* −, +, − */
        else
            sector = 5; /* −, +, + */
    }

    /* ------------------------------------------------------------------
     * Tempos dos vetores ativos (normalizados por Vdc e Ts)
     * Ta = tempo do primeiro vetor ativo do setor
     * Tb = tempo do segundo vetor ativo do setor
     * ------------------------------------------------------------------ */
    float Ta, Tb;

    switch (sector)
    {
    case 1:
        Ta = SQRT3 * (alpha - beta * SQRT3_INV);
        Tb = SQRT3 * (2.0f * beta * SQRT3_INV);
        break;
    case 2:
        Ta = SQRT3 * (alpha + beta * SQRT3_INV);
        Tb = SQRT3 * (-alpha + beta * SQRT3_INV);
        break;
    case 3:
        Ta = SQRT3 * (2.0f * beta * SQRT3_INV);
        Tb = SQRT3 * (-alpha - beta * SQRT3_INV);
        break;
    case 4:
        Ta = SQRT3 * (-alpha + beta * SQRT3_INV);
        Tb = SQRT3 * (-2.0f * beta * SQRT3_INV);
        break;
    case 5:
        Ta = SQRT3 * (-alpha - beta * SQRT3_INV);
        Tb = SQRT3 * (alpha - beta * SQRT3_INV);
        break;
    case 6:
        Ta = SQRT3 * (-2.0f * beta * SQRT3_INV);
        Tb = SQRT3 * (alpha + beta * SQRT3_INV);
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    /* Saturação (overmodulation) */
    float Tab = Ta + Tb;
    if (Tab > 1.0f)
    {
        Ta /= Tab;
        Tb /= Tab;
        Tab = 1.0f;
    }

    const float t0h = (1.0f - Tab) * 0.5f;

    /* ------------------------------------------------------------------
     * Duty cycles — derivados da sequência de chaveamento simétrica.
     * A fase que entra primeiro nos vetores ativos acumula Ta+Tb;
     * a que entra segundo acumula só Tb; a que fica nos vetores nulos
     * acumula apenas t0h.
     *
     * Sequências (000→Va→Vb→111→Vb→Va→000):
     *  S1: 000→100→110→111  U primeiro, V segundo, W nunca
     *  S2: 000→110→010→111  V primeiro, U segundo, W nunca  ← Ta↔Tb trocado vs S1
     *  S3: 000→010→011→111  V primeiro, W segundo, U nunca
     *  S4: 000→011→001→111  W primeiro, V segundo, U nunca
     *  S5: 000→001→101→111  W primeiro, U segundo, V nunca
     *  S6: 000→101→100→111  U primeiro, W segundo, V nunca
     * ------------------------------------------------------------------ */
    switch (sector)
    {
    case 1:
        duty->u = t0h + Ta + Tb;
        duty->v = t0h + Tb;
        duty->w = t0h;
        break;
    case 2:
        duty->u = t0h + Ta;
        duty->v = t0h + Ta + Tb;
        duty->w = t0h;
        break;
    case 3:
        duty->u = t0h;
        duty->v = t0h + Ta + Tb;
        duty->w = t0h + Tb;
        break;
    case 4:
        duty->u = t0h;
        duty->v = t0h + Ta;
        duty->w = t0h + Ta + Tb;
        break;
    case 5:
        duty->u = t0h + Tb;
        duty->v = t0h;
        duty->w = t0h + Ta + Tb;
        break;
    case 6:
        duty->u = t0h + Ta + Tb;
        duty->v = t0h;
        duty->w = t0h + Ta;
        break;
    }

    duty->u = clampf(duty->u, 0.0f, 1.0f);
    duty->v = clampf(duty->v, 0.0f, 1.0f);
    duty->w = clampf(duty->w, 0.0f, 1.0f);

    return ESP_OK;
}