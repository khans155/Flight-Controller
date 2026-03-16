#include "kalman.h"
#include "config.h"
#include "sensors.h"
#include <math.h>


void kalman_predict(Kalman1D *k, float accel, float dt) {
    // Integrate acceleration
    k->x += accel * dt;

    // Leak term — decays velocity toward zero
    // tau_decay: time constant in seconds. 1.0-3.0s works well for hover
    const float tau_decay = 2.0f;
    float decay = 1.0f - (dt / tau_decay);
    k->x *= decay;

    // Covariance growth
    k->p += k->q * dt * dt;
}

void kalman_correct(Kalman1D *k, float measurement, uint8_t quality) {
    // Below this quality, reject entirely
    if (quality < FLOW_QUALITY_MIN) return;
    // Clamp quality to a useful range first
    float q_clamped = fmaxf(0.0f, fminf((float)flow_quality, 150.0f));
    // Normalise to 0.0 - 1.0
    float q_norm = q_clamped / 150.0f;

    // R_base: baseline noise when quality is perfect
    // R_max:  noise when quality is just above minimum threshold
    // Low quality → high R → Kalman trusts measurement less
    const float R_base = 5.0f;
    const float R_max  = 200.0f;
    float R_dynamic = R_base + (1.0f - q_norm) * (R_max - R_base);

    // Innovation gate still applies, but uses dynamic R
    float innov = measurement - k->x;
    if (innov * innov > 9.0f * (k->p + R_dynamic)) return;

    float kg = k->p / (k->p + R_dynamic);
    k->x  += kg * innov;
    k->p  *= (1.0f - kg);
}