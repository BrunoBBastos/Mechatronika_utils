#include "pid.h"

void PID_init(PID_t *pid, PID_Config cfg) {
    pid->cfg = cfg;
    pid->st.i = 0.0f;
    pid->st.d = 0.0f;
    pid->st.prev_error = 0.0f;
    pid->st.prev_measurement = 0.0f;
    pid->st.output = 0.0f;
}

void PID_reset(PID_t *pid) {
    pid->st.i = 0.0f;
    pid->st.d = 0.0f;
    pid->st.prev_error = 0.0f;
    pid->st.prev_measurement = 0.0f;
    pid->st.output = 0.0f;
}

float PID_update(PID_t *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;

    // --- Proporcional ---
    float p = pid->cfg.Kp * error;

    // --- Integral ---
    // pid->st.i += pid->cfg.ki * T * erro; //(Euler) 

    pid->st.i += 0.5f * pid->cfg.Ki * pid->cfg.Ts * (error + pid->st.prev_error); // (Tustin)

    // --- Anti-windup ---
    if (pid->st.i > pid->cfg.lim_max) pid->st.i = pid->cfg.lim_max;
    else if (pid->st.i < pid->cfg.lim_min) pid->st.i = pid->cfg.lim_min;

    // --- Derivativo (Tustin + filtro LP) ---
    pid->st.d = ( (2.0f * pid->cfg.tau - pid->cfg.Ts) * pid->st.d
                - 2.0f * pid->cfg.Kd * (measurement - pid->st.prev_measurement) )
                / (2.0f * pid->cfg.tau + pid->cfg.Ts);

    float output = p + pid->st.i + pid->st.d;

    // saturação
    if (output > pid->cfg.lim_max) output = pid->cfg.lim_max;
    else if (output < pid->cfg.lim_min) output = pid->cfg.lim_min;

    pid->st.prev_error = error;
    pid->st.prev_measurement = measurement;
    pid->st.output = output;

    return output;
}