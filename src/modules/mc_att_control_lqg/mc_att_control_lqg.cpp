#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/set_point.h>
#include <uORB/uORB.h>

#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f

#define MAX_GYRO_COUNT 3

extern "C" __EXPORT int mc_att_control_lqg_main(int argc, char *argv[]);

#define N 6
#define M 2
#define P 2

struct mc_control {
    bool task_should_exit;
    int control_task;

    int ctrl_state_sub;
    int v_control_mode_sub;
    int params_sub;
    int manual_control_sp_sub;
    int armed_sub;
    int vehicle_status_sub;
    int motor_limits_sub;
    int battery_status_sub;
    int sensor_gyro_sub;
    int sensor_correction_sub;

    unsigned gyro_count;
    int selected_gyro;

    orb_advert_t actuators_0_pub;
    orb_advert_t set_point_pub;

    orb_id_t actuators_id;

    bool actuators_0_circuit_breaker_enabled;

    struct control_state_s ctrl_state;
    struct vehicle_control_mode_s v_control_mode;
    struct actuator_armed_s armed;
    struct actuator_controls_s actuators;
    struct manual_control_setpoint_s manual_control_sp;
    struct vehicle_status_s vehicle_status;
    struct multirotor_motor_limits_s motor_limits;
    struct battery_status_s battery_status;
    struct sensor_gyro_s sensor_gyro;
    struct sensor_correction_s sensor_correction;
    struct set_point_s set_point;

    perf_counter_t loop_perf;
    perf_counter_t controller_latency_perf;

    bool controller_is_initialized;

    float v_eqm;
    float A[N*N], B[N*M], C[P*N];
    float A_[P], B_[P];
    float G[M*N], G_[M*P], E[M*P], E_[M*P];
    float K[N*P], K_[N*N];
    float I1[P*N], I2[M*N], I3[M*P];

    float x[N];
    float r[P];
    float u[M];
    float x_prev[N];
    float r_prev[P];
    float y_prev[P];
    float s_prev[P];
    float h_prev[P];

    float bat_voltage;
};
static struct mc_control mc_control;

static void
initialize_controller(struct mc_control *const p, const float *s);

static bool
initialize_mc_control(struct mc_control *const p)
{
    FILE *fd = fopen("/fs/microsd/etc/lqg_gains", "rb");
    if (NULL == fd) return false;
    if (1 != fread(&p->v_eqm, sizeof p->v_eqm, 1, fd)) return false;
    if (N*N != fread(p->A , sizeof *p->A , N*N, fd)) return false;
    if (N*M != fread(p->B , sizeof *p->B , N*M, fd)) return false;
    if (P*N != fread(p->C , sizeof *p->C , P*N, fd)) return false;
    if (P != fread(p->A_, sizeof *p->A_, P, fd)) return false;
    if (P != fread(p->B_, sizeof *p->B_, P, fd)) return false;
    if (M*N != fread(p->G , sizeof *p->G , M*N, fd)) return false;
    if (M*P != fread(p->G_, sizeof *p->G_, M*P, fd)) return false;
    if (M*P != fread(p->E , sizeof *p->E , M*P, fd)) return false;
    if (M*P != fread(p->E_, sizeof *p->E_, M*P, fd)) return false;
    if (N*P != fread(p->K , sizeof *p->K , N*P, fd)) return false;
    if (N*N != fread(p->K_, sizeof *p->K_, N*N, fd)) return false;
    if (P*N != fread(p->I1, sizeof *p->I1, P*N, fd)) return false;
    if (M*N != fread(p->I2, sizeof *p->I2, M*N, fd)) return false;
    if (M*P != fread(p->I3, sizeof *p->I3, M*P, fd)) return false;
    fclose(fd);

    p->task_should_exit = false;
    p->control_task = -1;

    p->ctrl_state_sub = -1;
    p->v_control_mode_sub = -1;
    p->params_sub = -1;
    p->manual_control_sp_sub = -1;
    p->armed_sub = -1;
    p->vehicle_status_sub = -1;
    p->motor_limits_sub = -1;
    p->battery_status_sub = -1;
    p->sensor_gyro_sub = -1;
    p->sensor_correction_sub = -1;

    p->gyro_count = 1;
    p->selected_gyro = 0;

    p->actuators_0_pub = NULL;
    p->set_point_pub = NULL;
    p->actuators_id = ORB_ID(actuator_controls_0);

    p->actuators_0_circuit_breaker_enabled = false;

    p->loop_perf = perf_alloc(PC_ELAPSED, "mc_att_control_lqg");
    p->controller_latency_perf = perf_alloc_once(PC_ELAPSED, "ctrl_latency");

    p->ctrl_state = {};
    p->manual_control_sp = {};
    p->v_control_mode = {};
    p->actuators = {};
    p->armed = {};
    p->vehicle_status = {};
    p->motor_limits = {};
    p->set_point = {0};

    p->vehicle_status.is_rotary_wing = true;

    p->controller_is_initialized = false;

    p->bat_voltage = 11.1f;

    const float temp[P] = {0.0f};
    for (int i = 0; i < N; ++i) p->x[i] = 0.0f;
    initialize_controller(p, temp);

    for (int i = 0; i < 3; ++i) {
        p->sensor_correction.gyro_scale_0[i] = 1.0f;
        p->sensor_correction.gyro_scale_1[i] = 1.0f;
        p->sensor_correction.gyro_scale_2[i] = 1.0f;
    }
    return true;
}


static void
sgemv(const int m, int n, const float alpha, const float *A,
      const float *x, float *y)
{
    if (0.0f < alpha)
        do {
            const float x_j = *x++;
            int i = (m + (4 - 1)) >> 2;
            switch (m & (4 - 1)) {
            case 0: do { *y++ += (*A++)*x_j;
            case 3:      *y++ += (*A++)*x_j;
            case 2:      *y++ += (*A++)*x_j;
            case 1:      *y++ += (*A++)*x_j;
                    } while (--i != 0);
            }
        } while (y -= m, 0 != --n);
    else
        do {
            const float x_j = *x++;
            int i = (m + (4 - 1)) >> 2;
            switch (m & (4 - 1)) {
            case 0: do { *y++ -= (*A++)*x_j;
            case 3:      *y++ -= (*A++)*x_j;
            case 2:      *y++ -= (*A++)*x_j;
            case 1:      *y++ -= (*A++)*x_j;
                    } while (--i != 0);
            }
        } while (y -= m, 0 != --n);
}

static int
parameters_update(struct mc_control *const p)
{
    p->actuators_0_circuit_breaker_enabled = circuit_breaker_enabled(
        "CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);
    return OK;
}

static void
parameter_update_poll(struct mc_control *const p)
{
    bool updated;
    orb_check(p->params_sub, &updated);
    if (updated) {
        struct parameter_update_s param_update;
        orb_copy(ORB_ID(parameter_update), p->params_sub, &param_update);
        parameters_update(p);
    }
}

static void
vehicle_control_mode_poll(struct mc_control *const p)
{
    bool updated;
    orb_check(p->v_control_mode_sub, &updated);
    if (updated)
        orb_copy(ORB_ID(vehicle_control_mode),
                 p->v_control_mode_sub, &p->v_control_mode);
}

static void
vehicle_manual_poll(struct mc_control *const p)
{
    bool updated;
    orb_check(p->manual_control_sp_sub, &updated);
    if (updated)
        orb_copy(ORB_ID(manual_control_setpoint),
                 p->manual_control_sp_sub, &p->manual_control_sp);
}

static void
arming_status_poll(struct mc_control *const p)
{
    bool updated;
    orb_check(p->armed_sub, &updated);
    if (updated)
        orb_copy(ORB_ID(actuator_armed), p->armed_sub, &p->armed);
}


static void
vehicle_status_poll(struct mc_control *const p)
{
    bool updated;
    orb_check(p->vehicle_status_sub, &updated);
    if (updated)
        orb_copy(ORB_ID(vehicle_status),
                 p->vehicle_status_sub, &p->vehicle_status);
}

static void
vehicle_motor_limits_poll(struct mc_control *const p)
{
    bool updated;
    orb_check(p->motor_limits_sub, &updated);
    if (updated)
        orb_copy(ORB_ID(multirotor_motor_limits),
                 p->motor_limits_sub, &p->motor_limits);
}

static void
battery_status_poll(struct mc_control *const p)
{
    bool updated;
    orb_check(p->battery_status_sub, &updated);
    if (updated)
        orb_copy(ORB_ID(battery_status),
                 p->battery_status_sub, &p->battery_status);
}

static void
control_state_poll(struct mc_control *const p)
{
    bool updated;
    orb_check(p->ctrl_state_sub, &updated);
    if (updated)
        orb_copy(ORB_ID(control_state), p->ctrl_state_sub, &p->ctrl_state);
}

static void
sensor_correction_poll(struct mc_control *const p)
{
    bool updated;
    orb_check(p->sensor_correction_sub, &updated);
    if (updated)
        orb_copy(ORB_ID(sensor_correction),
                 p->sensor_correction_sub, &p->sensor_correction);
    if (p->sensor_correction.selected_gyro_instance < p->gyro_count)
        p->selected_gyro = p->sensor_correction.selected_gyro_instance;
}

static inline void
get_euler_angles_from_att_quaternion(const float *q, float *z)
{
    z[0] = atan2f(
        2.0f*(q[2]*q[3] + q[0]*q[1]), 2.0f*(q[0]*q[0] + q[3]*q[3]) - 1.0f);
    z[1] = asinf(2.0f*(q[1]*q[3] - q[0]*q[2]));
}

static void
initialize_controller(struct mc_control *const p, const float *s)
{
    for (int i = 0; i < N; i++) 
        p->x_prev[i] = 0.0f;
    for (int i = 0; i < P; i++) {
        p->r[i]      = 0.0f;
        p->r_prev[i] = 0.0f;
        p->s_prev[i] = 0.0f;
        p->y_prev[i] = 0.0f;
        p->h_prev[i] = 0.0f;
        p->u[i]      = 0.0f;
    }
    sgemv(P, N,  1.0f, p->I1, p->x, p->r);
    sgemv(M, N,  1.0f, p->I2, p->x, p->u);
    sgemv(M, P, -1.0f, p->I3,    s, p->u);
}

static void
update_bat_voltage(struct mc_control *const p)
{
    static uint64_t timer_cnt_prev = 0;
    if (p->battery_status.connected && 0 != timer_cnt_prev) {
#define BAT_VOLTAGE_MEAS_PERIOD 1 * 1000000
#define BAT_VOLTAGE_MEAS_MAX 40
        const uint64_t timer_cnt = hrt_absolute_time();
        const uint64_t time_elapsed = timer_cnt - timer_cnt_prev;
        if (time_elapsed > BAT_VOLTAGE_MEAS_PERIOD) {
            static float bat_voltage_meas_sum = 0.0f;
            static int bat_voltage_meas_cnt = 0;
            bat_voltage_meas_sum += p->battery_status.voltage_filtered_v;
            if (BAT_VOLTAGE_MEAS_MAX < ++bat_voltage_meas_cnt) {
                p->bat_voltage = bat_voltage_meas_sum/bat_voltage_meas_cnt;
                timer_cnt_prev = timer_cnt;
                bat_voltage_meas_sum = 0.0f;
                bat_voltage_meas_cnt = 0;
            }
        }
    } else {
        if (p->battery_status.connected) {
            p->bat_voltage = p->battery_status.voltage_filtered_v;
            timer_cnt_prev = hrt_absolute_time();
        }
#define BAT_VOLTAGE_NOM 11.1f
        else p->bat_voltage = BAT_VOLTAGE_NOM;
    }
}

static void
check_bounds_controls(struct mc_control *p)
{
    float v[] = {
        p->v_eqm + 0.5f*p->u[0], p->v_eqm + 0.5f*p->u[1],
        p->v_eqm - 0.5f*p->u[0], p->v_eqm - 0.5f*p->u[1],
    };
#define V_MIN 0.8f
    if (v[0] > p->bat_voltage) {
        v[0] = p->bat_voltage;
        p->actuators.control[0] = 1.0f;
    } else {
        v[0] = (v[0] < V_MIN)? V_MIN: v[0];
        p->actuators.control[0] = 2.0f*(v[0]/p->bat_voltage) - 1.0f;
    }
    if (v[1] > p->bat_voltage) {
        v[1] = p->bat_voltage;
        p->actuators.control[1] = 1.0f;
    } else {
        v[1] = (v[1] < V_MIN)? V_MIN: v[1];
        p->actuators.control[1] = 2.0f*(v[1]/p->bat_voltage) - 1.0f;
    }
    if (v[2] > p->bat_voltage) {
        v[2] = p->bat_voltage;
        p->actuators.control[2] = 1.0f;
    } else {
        v[2] = (v[2] < V_MIN)? V_MIN: v[2];
        p->actuators.control[2] = 2.0f*(v[2]/p->bat_voltage) - 1.0f;
    }
    if (v[3] > p->bat_voltage) {
        v[3] = p->bat_voltage;
        p->actuators.control[3] = 1.0f;
    } else {
        v[3] = (v[3] < V_MIN)? V_MIN: v[3];
        p->actuators.control[3] = 2.0f*(v[3]/p->bat_voltage) - 1.0f;
    }
    p->u[0] = v[0] - v[2];
    p->u[1] = v[1] - v[3];
}

static inline void
kalman_filter_update_estimate(struct mc_control *const p, const float *z)
{
    float y[N];
    memcpy(y, p->x, sizeof y);
    sgemv(N, N, -1.0f, p->K_, y, p->x);
    sgemv(N, P,  1.0f, p->K , z, p->x);
}

static inline void
kalman_filter_propagate_states(struct mc_control *const p)
{
    memcpy(p->x_prev, p->x, sizeof p->x);
    p->y_prev[0] = p->x_prev[4];
    p->y_prev[1] = p->x_prev[5];

    for (int i = 0; i < N; ++i) p->x[i] = 0.0f;
    sgemv(N, N, 1.0f, p->A, p->x_prev, p->x);
    sgemv(N, M, 1.0f, p->B, p->u, p->x);
}

static inline void
cgt_propagate_model(struct mc_control *const p, const float *set_point)
{
    memcpy(p->r_prev, p->r, sizeof p->r);
    memcpy(p->s_prev, set_point, sizeof p->s_prev);
    p->h_prev[0] = p->r_prev[0];
    p->h_prev[1] = p->r_prev[1];
    p->r[0] = p->A_[0]*p->r_prev[0] + p->B_[0]*p->s_prev[0];
    p->r[1] = p->A_[1]*p->r_prev[1] + p->B_[1]*p->s_prev[1];
}

static inline void
lqg_compute_controls(struct mc_control *const p, const float *s)
{
    sgemv(M, N, -1.0f, p->G , p->x, p->u);
    sgemv(M, N,  1.0f, p->G , p->x_prev, p->u);
    sgemv(M, P, -1.0f, p->G_, p->y_prev, p->u);
    sgemv(M, P,  1.0f, p->G_, p->h_prev, p->u);
    sgemv(M, P, -1.0f, p->E , p->r_prev, p->u);
    sgemv(M, P,  1.0f, p->E , p->r, p->u);
    sgemv(M, P, -1.0f, p->E_, p->s_prev, p->u);
    sgemv(M, P,  1.0f, p->E_, s, p->u);
}

static void
get_euler_angles_set_point(const float *set_point_raw, float *set_point)
{
    static float _set_point[2] = {0.0f};
#define NR_MEAS 70
    static int meas_cnt = 0;
    if (NR_MEAS > meas_cnt) {
        _set_point[0] += set_point_raw[0];
        _set_point[1] += set_point_raw[1];
        if (NR_MEAS == ++meas_cnt) {
            set_point[0] = _set_point[0] /= meas_cnt;
            set_point[1] = _set_point[1] /= meas_cnt;
        } else {
            set_point[0] = _set_point[0]/meas_cnt;
            set_point[1] = _set_point[1]/meas_cnt;
        }
    } else {
        _set_point[0] += (set_point_raw[0] - _set_point[0])/(NR_MEAS + 1);
        _set_point[1] += (set_point_raw[1] - _set_point[1])/(NR_MEAS + 1);
        set_point[0] = _set_point[0];
        set_point[1] = _set_point[1];
    }
}

extern "C" int commander_main(int argc, char *argv[]);
static void
task_main(int argc, char *argv[])
{
    struct mc_control *const p = &mc_control;
#define NOOP (void)0
    p->ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    p->v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    p->params_sub = orb_subscribe(ORB_ID(parameter_update));
    p->manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    p->armed_sub = orb_subscribe(ORB_ID(actuator_armed));
    p->vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    p->motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
    p->battery_status_sub = orb_subscribe(ORB_ID(battery_status));
#define MIN(a, b) (((a) < (b))? (a): (b))
    p->gyro_count = MIN(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);
    if (0 == p->gyro_count) p->gyro_count = 1;
    p->sensor_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
    p->sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));
    parameters_update(p);
    /* wakeup source: gyro data */
    px4_pollfd_struct_t poll_fds = {};
    poll_fds.events = POLLIN;
    poll_fds.fd = p->sensor_gyro_sub;

    bool motors_were_armed = false;
    /* int iter_cnt = -2500; */
    while (false == p->task_should_exit) {
        int pret = px4_poll(&poll_fds, 1, 100);
        /* timed out - periodic check for _task_should_exit */
        if (0 == pret) continue;
        if (0 > pret) {
            warn("mc att ctrl: poll error %d, %d", pret, errno);
            usleep(100000);
            continue;
        }
        perf_begin(p->loop_perf);
        if (poll_fds.revents & POLLIN) {
            /* TODO Handle delays  */
            orb_copy(ORB_ID(sensor_gyro), p->sensor_gyro_sub, 
                     &p->sensor_gyro);

            parameter_update_poll(p);
            vehicle_control_mode_poll(p);
            arming_status_poll(p);
            vehicle_manual_poll(p);
            vehicle_status_poll(p);
            vehicle_motor_limits_poll(p);
            battery_status_poll(p);
            control_state_poll(p);
            sensor_correction_poll(p);

            if (p->v_control_mode.flag_control_attitude_enabled) {
                float euler_angles[2];
                get_euler_angles_from_att_quaternion(
                    p->ctrl_state.q, euler_angles);
#define ROLL_BOUND         ((float)(25.0*M_PI/180.0))
#define ROLL_BOUND_DISARM  ((float)(35.0*M_PI/180.0))
#define PITCH_BOUND        ((float)(25.0*M_PI/180.0))
#define PITCH_BOUND_DISARM ((float)(35.0*M_PI/180.0))
                if (ROLL_BOUND_DISARM  < fabsf(euler_angles[0]) ||
                    PITCH_BOUND_DISARM < fabsf(euler_angles[1])) {
                    if (true == p->armed.armed) {
                        const char *command[] = {"", "disarm"};
                        (void)commander_main(2, (char **)command);
                    }
                }
#if 0
                static int sig_state = 0;
                if (true == p->armed.armed && 1000 < ++iter_cnt) {
                    sig_state ^= 1;
                    iter_cnt = 0;
                }
                p->set_point.raw[0] = 0.0f;// sig_state*ROLL_BOUND;
                p->set_point.raw[1] = -sig_state*PITCH_BOUND;
                p->set_point.filtered[0] = p->set_point.raw[0]; 
                p->set_point.filtered[1] = p->set_point.raw[1]; 
#endif
                p->set_point.raw[0] = ROLL_BOUND*p->manual_control_sp.y;
                p->set_point.raw[1] = PITCH_BOUND*p->manual_control_sp.x;
                get_euler_angles_set_point(
                    p->set_point.raw, p->set_point.filtered);
                if (false == motors_were_armed && true == p->armed.armed) {
                    const float angular_velocity[] = {
                        p->ctrl_state.roll_rate, -p->ctrl_state.pitch_rate
                    };
                    p->x[0] = angular_velocity[0];
                    p->x[1] = angular_velocity[1];
                    p->x[2] = p->x[3] = 0.0f;
                    p->x[4] = euler_angles[0];
                    p->x[5] = euler_angles[1];
                    initialize_controller(p, p->set_point.filtered);
                } else {
                    static uint64_t _ = 0;
                    p->set_point.sampling_time = (hrt_absolute_time() - _)/1e6f;
                    _ = hrt_absolute_time();
                    kalman_filter_update_estimate(p, euler_angles);
                }
                motors_were_armed = p->armed.armed;

                lqg_compute_controls(p, p->set_point.filtered);
                update_bat_voltage(p);
                check_bounds_controls(p);
                p->actuators.timestamp_sample = p->ctrl_state.timestamp;

                p->actuators.timestamp = hrt_absolute_time();
                p->set_point.timestamp = p->actuators.timestamp;
                if (false == p->actuators_0_circuit_breaker_enabled) {
                    if (NULL != p->actuators_0_pub) {
                        orb_publish(p->actuators_id, p->actuators_0_pub,
                                    &p->actuators);
                        perf_end(p->controller_latency_perf);
                    } else if (NULL != p->actuators_id) {
                        p->actuators_0_pub = orb_advertise(
                            p->actuators_id, &p->actuators);
                    } else NOOP;
                }
                if (NULL != p->set_point_pub) {
                    orb_publish(ORB_ID(set_point), 
                                p->set_point_pub, &p->set_point);
                } else {
                    p->set_point_pub = orb_advertise(
                        ORB_ID(set_point), &p->set_point);
                }
                kalman_filter_propagate_states(p);
                cgt_propagate_model(p, p->set_point.filtered);
            }
        }
        perf_end(p->loop_perf);
    }
    p->control_task = -1;
}

static int
start(struct mc_control *const p)
{
    ASSERT(-1 == p->control_task);
    p->control_task = px4_task_spawn_cmd(
        "mc_att_control_lqg", SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5,
        1700, (px4_main_t)task_main, NULL);
    if (0 > p->control_task) {
        warn("task start failed");
        return -errno;
    }
    return OK;
}

int
mc_att_control_lqg_main(int argc, char *argv[])
{
    static bool program_is_running = false;
    if (argc < 2) {
        warnx("usage: lqg {start|stop|status}");
        return 1;
    }
    if (!strcmp(argv[1], "start")) {
        if (program_is_running) {
            warnx("already running");
            return 1;
        }
        if(!initialize_mc_control(&mc_control)) {
            warnx("error opening lqg_gains file");
            return 1;
        }
        program_is_running = true;
        if (OK != start(&mc_control)) {
            program_is_running = false;
            warnx("start failed");
            return 1;
        }
        return OK;
    }
    if (!strcmp(argv[1], "stop")) {
        if (!program_is_running) {
            warnx("not running");
            return 1;
        }
        program_is_running = false;
        mc_control.task_should_exit = true;
        return 0;
    }
    if (!strcmp(argv[1], "status")) {
        if (program_is_running) {
            warnx("running");
            return 0;

        } else {
            warnx("not running");
            return 1;
        }
    }
    warnx("unrecognized command");
    return 1;
}
