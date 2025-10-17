#pragma once // substitui os defines/include guards

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Ts;
    float tau;
    float lim_min;
    float lim_max;
} PID_Config;

typedef struct {
    float i;
    float d;
    float prev_error;
    float prev_measurement;
    float output;
} PID_State;

typedef struct {
    PID_Config cfg;
    PID_State st;
} PID_t;

void PID_init(PID_t *pid, PID_Config cfg);
void PID_reset(PID_t *pid);
float PID_update(PID_t *pid, float setpoint, float measurement);