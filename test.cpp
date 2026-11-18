// ---- tunables (OUTPUT units: rad, rad/s, rad/s^2) ----
#define DT_POS     (1.0f / 1250.0f)
#define KP_POS     60.0f
#define V_MAX      10.0f              // your cruise speed
#define A_DECEL    30.0f              // braking the ramp allows
#define A_PLAN     (0.8f * A_DECEL)   // braking the sqrt cap plans for
#define A_ACCEL    60.0f              // speeding up, away from target
#define NEAR_ZONE  0.05f              // rad: ~2-5x backlash
#define DB_IN      0.002f
#define DB_OUT     0.006f
#define VEL_DB     0.02f

static float vel_cmd = 0.0f;          // ramped command, keep between ticks
static bool  holding = false;

// returns velocity command in MOTOR rad/s for the velocity loop
float position_tick(float target, float pos_fb, float vel_meas_out)
{
    float e  = target - pos_fb;
    float ae = fabsf(e);

    // 1. deadband with hysteresis
    if (holding && ae > DB_OUT) holding = false;
    if (!holding && ae < DB_IN && fabsf(vel_meas_out) < VEL_DB) holding = true;

    // 2. P with sqrt cap: never ask for more speed than you can stop from
    float vraw = 0.0f;
    if (!holding) {
        float e_eff  = fmaxf(ae - DB_IN, 0.0f);
        float v_lin  = KP_POS * e_eff;
        float v_stop = sqrtf(2.0f * A_PLAN * e_eff);
        vraw = copysignf(fminf(fminf(v_lin, v_stop), V_MAX), e);
    }

    // 3. ramp: gentle when braking OR near target (covers the reverse)
    float dv     = vraw - vel_cmd;
    bool braking = (vel_cmd * dv) < 0.0f;
    bool near    = ae < NEAR_ZONE;
    float lim    = ((braking || near) ? A_DECEL : A_ACCEL) * DT_POS;
    vel_cmd += clampf(dv, -lim, lim);

    return vel_cmd * GEAR_RATIO;      // output rad/s -> motor rad/s
}
