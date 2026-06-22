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
    {
        return ESP_ERR_INVALID_ARG;
    }

    const float alpha = ab->alpha;
    const float beta = ab->beta;

    /* ------------------------------------------------------------------
     *
     * O plano αβ é dividido em 6 setores de 60°. Em vez de atan2(),
     * usam-se três grandezas auxiliares cujo sinal identifica o setor:
     *
     *   v_reg1 =  β                          (fronteira S1/S6 e S3/S4)
     *   v_reg2 = (√3·α − β) / 2             (fronteira S1/S2 e S4/S5)
     *   v_reg3 = (−√3·α − β) / 2            (fronteira S2/S3 e S5/S6)
     *
     * Tabela de sinais:
     *   Setor | v_reg1 | v_reg2 | v_reg3
     *     1   |   +    |   −    |   −
     *     2   |   +    |   +    |   (qualquer)
     *     3   |   −    |   +    |   −
     *     4   |   −    |   +    |   +
     *     5   |   −    |   −    |   (qualquer)
     *     6   |   +    |   −    |   +
     * ------------------------------------------------------------------ */
    const float v_reg1 = beta;
    const float v_reg2 = (SQRT3 * alpha - beta) * 0.5f;
    const float v_reg3 = (-SQRT3 * alpha - beta) * 0.5f;

    int sector = 0;

    if (v_reg1 >= 0.0f)
    {
        if (v_reg2 >= 0.0f)
        {
            /* v_reg1 ≥ 0, v_reg2 ≥ 0 → setor 2 (v_reg3 irrelevante) */
            sector = 2;
        }
        else if (v_reg3 >= 0.0f)
        {
            /* v_reg1 ≥ 0, v_reg2 < 0, v_reg3 ≥ 0 → setor 6 */
            sector = 6;
        }
        else
        {
            /* v_reg1 ≥ 0, v_reg2 < 0, v_reg3 < 0 → setor 1 */
            sector = 1;
        }
    }
    else /* v_reg1 < 0 */
    {
        if (v_reg2 >= 0.0f)
        {
            if (v_reg3 >= 0.0f)
            {
                /* v_reg1 < 0, v_reg2 ≥ 0, v_reg3 ≥ 0 → setor 4 */
                sector = 4;
            }
            else
            {
                /* v_reg1 < 0, v_reg2 ≥ 0, v_reg3 < 0 → setor 3 */
                sector = 3;
            }
        }
        else
        {
            /* v_reg1 < 0, v_reg2 < 0 → setor 5 (v_reg3 irrelevante) */
            sector = 5;
        }
    }

    if (sector == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    float Ta = 0.0f; /* Tempo do primeiro vetor ativo  */
    float Tb = 0.0f; /* Tempo do segundo vetor ativo   */

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
    float Tab = Ta + Tb;

    if (Tab > 1.0f)
    {
        Ta = Ta / Tab;
        Tb = Tb / Tab;
        Tab = 1.0f;
    }

    const float T0 = 1.0f - Tab;
    const float t_half0 = T0 * 0.5f;

    switch (sector)
    {
    case 1:
        duty->u = t_half0;
        duty->v = duty->u + Ta;
        duty->w = duty->v + Tb;
        break;
    case 2:
        duty->v = t_half0;
        duty->u = duty->v + Tb;
        duty->w = duty->u + Ta;
        break;
    case 3:
        duty->v = t_half0;
        duty->w = duty->v + Ta;
        duty->u = duty->w + Tb;
        break;
    case 4:
        duty->w = t_half0;
        duty->v = duty->w + Tb;
        duty->u = duty->v + Ta;
        break;
    case 5:
        duty->w = t_half0;
        duty->u = duty->w + Ta;
        duty->v = duty->u + Tb;
        break;
    case 6:
        duty->u = t_half0;
        duty->w = duty->u + Tb;
        duty->v = duty->w + Ta;
        break;
    }

    duty->u = clampf(duty->u, 0.0f, 1.0f);
    duty->v = clampf(duty->v, 0.0f, 1.0f);
    duty->w = clampf(duty->w, 0.0f, 1.0f);

    return ESP_OK;
}