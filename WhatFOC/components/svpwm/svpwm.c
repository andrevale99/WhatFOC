
#include "svpwm.h"

#include <math.h>

#define M_PI 3.14159265358979323846f
#define SQRT3 1.73205080756887729353f

esp_err_t svpwm_calculate(
    const svpwm_ab_t *ab,
    svpwm_duty_t *duty)
{
    // Limitar alpha e beta para evitar valores fora do círculo de referência
    float alpha = ab->alpha;
    float beta = ab->beta;

    // Calcular os ângulos dos vetores de referência
    float angle = atan2f(beta, alpha);
    if (angle < 0)
    {
        angle += 2 * M_PI; // Garantir que o ângulo esteja entre 0 e 2π
    }

    // Determinar o setor do espaço vetorial
    int sector = (int)(angle / (M_PI / 3)) + 1; // Setores de 1 a 6

    // Calcular os duty cycles com base no setor
    float T1, T2;
    switch (sector)
    {
    case 1:
        T1 = SQRT3 * beta;
        T2 = alpha - beta / SQRT3;
        break;
    case 2:
        T1 = alpha + beta / SQRT3;
        T2 = SQRT3 * beta - alpha;
        break;
    case 3:
        T1 = alpha + beta / SQRT3;
        T2 = -alpha - beta / SQRT3;
        break;
    case 4:
        T1 = -SQRT3 * beta;
        T2 = -alpha - beta / SQRT3;
        break;
    case 5:
        T1 = -alpha + beta / SQRT3;
        T2 = -SQRT3 * beta + alpha;
        break;
    case 6:
        T1 = -alpha + beta / SQRT3;
        T2 = alpha + beta / SQRT3;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    // Normalizar os duty cycles para o intervalo [0, 1]
    float max_T = fmaxf(T1, T2);
    if (max_T > 0)
    {
        duty->u = (T1 / max_T) * 0.5f + 0.5f; // Normalizar para [0, 1]
        duty->v = (T2 / max_T) * 0.5f + 0.5f;
        duty->w = 1.0f - duty->u - duty->v;
    }
    else
    {
        duty->u = duty->v = duty->w = 0.0f;
    }

    return ESP_OK;
}