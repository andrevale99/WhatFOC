#ifndef SVPWM_H
#define SVPWM_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float alpha;
        float beta;
    } svpwm_ab_t;

    typedef struct
    {
        float u;
        float v;
        float w;
    } svpwm_duty_t;

    /**
     * @brief Calcula os duty cycles SVPWM a partir do vetor αβ.
     *
     * Entradas:
     *   alpha, beta normalizados.
     *
     * Saídas:
     *   duty_u, duty_v, duty_w normalizados entre 0.0 e 1.0.
     */
    esp_err_t svpwm_calculate(
        const svpwm_ab_t *ab,
        svpwm_duty_t *duty);

#ifdef __cplusplus
}
#endif

#endif /* SVPWM_H */