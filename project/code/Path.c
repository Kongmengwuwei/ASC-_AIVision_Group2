#include "Path.h"
#include "Motor.h"
#include "pid.h"
#include "zf_device_ips200.h"
#include <math.h>
#include "Attitude.h"



#ifndef M_PI
#define M_PI 3.1415926
#endif

#define DX_FAST_DISTANCE 0.6f  // 快速段切换阈值 m
#define DX_SLOW_DISTANCE 0.2f  // 慢速段切换阈值 m
#define DY_FAST_DISTANCE 0.6f  // 快速段切换阈值 m
#define DY_SLOW_DISTANCE 0.2f  // 慢速段切换阈值 m

#define CAR_DRIVER_MAX_SPEED 200.0f // 最大驱动速度 cm/s
#define CAR_DRIVER_MAX_ACCELERATE 2.0f // 最大驱动加速度 cm/s

#define CAR_DRIVER_MIN_SPEED 40.0f  // 最小驱动速度 cm/s

#define START_RAMP_ACCEL_CMSS 140.0f
#define QUICK_STOP_DISTANCE 0.12f
#define QUICK_STOP_SPEED_GAIN 6.0f
#define QUICK_STOP_MIN_SPEED 30.0f
#define PRESTART_OFFSET_RESOLUTION_M 0.01f

// 路径跟随核心状态（拆成独立变量，便于直接看和改）
//pf为pathfollow的简写
static size_t pf_steps = 0;                 // 当前路径总点数
static const Point *pf_path = 0;            // 当前路径数组首地址（路径点序列）
static size_t pf_idx = 0;                   // 当前正在跟随的目标点索引
static float pf_grid_m = 0.0f;              // 网格分辨率（每格多少米）
static float pf_pulses_per_meter = 0.0f;    // 编码器脉冲到米的换算系数
static float pf_pos_tol_m = 0.03f;          // 到点位置容差（米）
static float pf_yaw_tol_deg = 3.0f;         // 航向容差（度，预留）
static float pf_max_v_mps = 2.5f;           // 路径跟随最大线速度（m/s）
static float pf_max_w_rad = 360.0f;         // 路径跟随最大角速度（deg/s 语义）
static float actual_x_position = 0.0f;      // 里程计估计的当前 X 位置（米）
static float actual_y_position = 0.0f;      // 里程计估计的当前 Y 位置（米）
static float pf_yaw_deg = 0.0f;             // 当前航向角（度）
static uint8 pf_active = 0;                 // 路径跟随使能标志（1 运行，0 停止）
tagPID_T pid_world_x;     // X 轴位置环 PID
tagPID_T pid_world_y;     // Y 轴位置环 PID
tagPID_T pid_stay;         // 定点保持 PID
tagPID_T pid_yaw;         // 航向角速度 PID
tagPID_T pid_accel_yaw;         // 航向角加速度 PID
static PIDInitStruct pid_world_init;
static PIDInitStruct pid_stay_init;
static PIDInitStruct pid_yaw_init;
static PIDInitStruct pid_accel_yaw_init;
uint8 car_direction = 0;  // 当前运动方向0停 1X 2Y 3XY
uint8 wait_stop = 0;
static float g_start_ramp_cmps = 0.0f;      // 软启动当前爬升速度（cm/s）
static int8 g_start_ramp_sign = 0;          // 软启动方向符号（-1/0/1）
static Point g_prestart_offset_path[3];     // 发车偏移用的临时路径缓存（最多3点

// 对称限幅：把值限制到 [-limit, limit]
static float clamp_sym(float v, float limit)
{
    if (v > limit)
    {
        return limit;
    }
    if (v < -limit)
    {
        return -limit;
    }
    return v;
}

static void path_follow_update_pid_limits(void)
{
    // 当速度上限变化时，同步更新 PID 输出上限
    pid_world_init.fMax_Iout = pf_max_v_mps * 100.0f;
    pid_world_init.fMax_Out = pf_max_v_mps * 100.0f;
    PID_Update(&pid_world_x, &pid_world_init);
    PID_Update(&pid_world_y, &pid_world_init);

    pid_yaw_init.fMax_Iout = pf_max_w_rad * 20.0f;
    pid_yaw_init.fMax_Out = pf_max_w_rad * 20.0f;
    PID_Update(&pid_yaw, &pid_yaw_init);
}
static float wrap_pi(float rad)
{
    while (rad > (float)M_PI)
    {
        rad -= 2.0f * (float)M_PI;
    }
    while (rad < -(float)M_PI)
    {
        rad += 2.0f * (float)M_PI;
    }
    return rad;
}
static int8_t fsignal(float num)//判断符号
{
    if (num > 0)
        return 1;
    else if (num < 0)
        return -1;
    else
        return 0;
}

static float f_my_max(float a, float b)
{
    float a_abs = fabsf(a);
    float b_abs = fabsf(b);
    return (a_abs > b_abs) ? a_abs : b_abs;
}

static float f_my_min(float a, float b)
{
    float a_abs = fabsf(a);
    float b_abs = fabsf(b);
    return (a_abs < b_abs) ? a_abs : b_abs;
}

static void path_follow_reset_soft_start(void)
{
    g_start_ramp_cmps = 0.0f;
    g_start_ramp_sign = 0;
}



static void path_follow_reset_soft_start_state(void)
{
    path_follow_reset_soft_start();

}

static float path_follow_apply_soft_start(float target_speed_cmps,
                                          float remain_distance_m,
                                          float *ramp_speed_cmps,
                                          int8 *sign_state)//防止突变，软启动，不是发车函数
{
    float cmd_abs = fabsf(target_speed_cmps);          // 目标速度取绝对值，只看幅值
    int8 cmd_sign = fsignal(target_speed_cmps);        // 目标速度取符号，记录方向

    if (cmd_sign == 0 || cmd_abs < 1.0f)               // 速度太小或为 0
    {
        path_follow_reset_soft_start(); // 直接清空软启动状态
        return 0.0f;                                   // 输出 0，防止抖动
    }
    if (remain_distance_m < QUICK_STOP_DISTANCE)       // 已经进入近点区域
    {
        path_follow_reset_soft_start(); // 不做软启动了
        return target_speed_cmps;                      // 直接按目标速度走
    }

    if (*sign_state != 0 && *sign_state != cmd_sign)   // 如果方向发生反转
    {
        *ramp_speed_cmps = 0.0f;                       // 软启动速度重置到 0
    }
    if (*sign_state == 0)                              // 第一次进入软启动
    {
        *ramp_speed_cmps = 0.0f;                       // 从 0 开始爬升
    }
    *sign_state = cmd_sign;                            // 记录当前方向给下一周期对比

    if (*ramp_speed_cmps < cmd_abs)                    // 还没爬升到目标速度
    {
        float ramp_step = START_RAMP_ACCEL_CMSS / (float)PID_RATE; // 每周期可增长速度
        *ramp_speed_cmps += ramp_step;                 // 累加软启动速度
        if (*ramp_speed_cmps > cmd_abs)                // 防止超过目标幅值
        {
            *ramp_speed_cmps = cmd_abs;                // 到顶就卡住
        }
        return (float)cmd_sign * (*ramp_speed_cmps);   // 恢复方向后输出
    }

    *ramp_speed_cmps = cmd_abs;                        // 已达到目标速度，保持
    return target_speed_cmps;                          // 直接返回目标速度
}

static float path_follow_apply_quick_stop_limit(float target_speed_cmps, float remain_distance_m)//接近目标时做减速
{
    // 靠近目标点时，按剩余距离动态压低速度上限
    float remain_abs = fabsf(remain_distance_m);
    if (remain_abs >= QUICK_STOP_DISTANCE)
    {
        return target_speed_cmps;
    }

    float speed_cap = QUICK_STOP_SPEED_GAIN * remain_abs * 100.0f;
    if (speed_cap < QUICK_STOP_MIN_SPEED)
    {
        speed_cap = QUICK_STOP_MIN_SPEED;
    }
    return (float)fsignal(target_speed_cmps) * f_my_min(target_speed_cmps, speed_cap);
}

void path_follow_init(float grid_size_m, float pulses_per_meter)
{
    // 初始化路径状态、里程计状态、PID 参数
    pf_grid_m = grid_size_m;
    pf_pulses_per_meter = pulses_per_meter;
    pf_pos_tol_m = 0.03f;   // 位置容差，可按需要调整
    pf_yaw_tol_deg = 3.0f;  // 航向容差
    pf_max_v_mps = 2.5f;    // 最大线速度
    pf_max_w_rad = 360.0f;    // 角速度上限
    actual_x_position = 0.0f;
    actual_y_position = 0.0f;
    pf_yaw_deg = 0.0f;
    pf_path = NULL;
    pf_steps = 0;
    pf_idx = 0;
    pf_active = 0;

    pid_world_init.fKp = 2.2f;  // 位置误差到速度输出
    pid_world_init.fKi = 0.0f;  //位置环
    pid_world_init.fKd = 1.8f;
    pid_world_init.fMax_Iout = pf_max_v_mps*100.0f;
    pid_world_init.fMax_Out = pf_max_v_mps*100.0f;
    pid_world_init.alpha = 0.9f;

    pid_stay_init.fKp = 0.7f;  // 位置误差到速度输出
    pid_stay_init.fKi = 0.0f;
    pid_stay_init.fKd = 0.3f;
    pid_stay_init.fMax_Iout = 200.0f;
    pid_stay_init.fMax_Out = 200.0f;
    pid_stay_init.alpha = 0.9f;

    pid_yaw_init.fKp = 6.2f;  // 航向误差到角速度输出
    pid_yaw_init.fKi = 0.0f;
    pid_yaw_init.fKd = 10.5f;
    pid_yaw_init.fMax_Iout = pf_max_w_rad;
    pid_yaw_init.fMax_Out = pf_max_w_rad;
    pid_yaw_init.alpha = 0.9f;

    pid_accel_yaw_init.fKp = 1.2f;  // 航向误差到角速度输出
    pid_accel_yaw_init.fKi = 0.0f;  //角速度环，控制在0度用的
    pid_accel_yaw_init.fKd = 2.1f;
    pid_accel_yaw_init.fMax_Iout = pf_max_w_rad;
    pid_accel_yaw_init.fMax_Out = pf_max_w_rad;
    pid_accel_yaw_init.alpha = 0.9f;

    PID_Init(&pid_world_x, &pid_world_init);
    PID_Init(&pid_stay, &pid_stay_init);
    PID_Init(&pid_world_y, &pid_world_init);
    PID_Init(&pid_yaw, &pid_yaw_init);
    PID_Init(&pid_accel_yaw, &pid_accel_yaw_init);
}

// 重置里程计起点
void path_follow_reset_pose(float x_m, float y_m, float yaw_deg)
{
    // 外部可直接重置里程计坐标（例如视觉矫正后）
    actual_x_position = x_m;
    actual_y_position = y_m;
    pf_yaw_deg = yaw_deg;
}

void path_follow_set_external_position(float x_m, float y_m, uint8 valid)
{
    // 预留接口：当前项目未使用外部位置融合
    (void)x_m;
    (void)y_m;
    (void)valid;
}

// 设置路径并复位索引
void path_follow_set_path(const Point *path, size_t steps)
{
    // 切换新路径时先清软启动，避免沿用旧路径的加速状态
    path_follow_reset_soft_start_state();
    pf_path = path;
    pf_steps = steps;
    pf_idx = 0;
    if (path && steps > 0)
    {
        actual_x_position = path[0].row * pf_grid_m;
        actual_y_position = path[0].col * pf_grid_m;
        pf_active = 1;
    }
    else
    {
        pf_active = 0;
    }
}


void path_follow_start_offset_move(float delta_x_m, float delta_y_m)//启动函数，刚开始的时候跑一段短距离，显示地图
{
    // 小于该阈值视为“无需偏移”
    const float eps_m = 0.001f;
    // 起点坐标
    float start_x_m = actual_x_position;
    float start_y_m = actual_y_position;
    // 中间点：先沿 X 方向移动
    float mid_x_m = start_x_m + delta_x_m;
    float mid_y_m = start_y_m;
    // 终点：在中间点基础上再沿 Y 方向移动
    float end_x_m = mid_x_m;
    float end_y_m = start_y_m + delta_y_m;
    // 默认至少有起点
    size_t steps = 1;

    // X/Y 偏移都很小：直接清空路径并退出
    if (fabsf(delta_x_m) <= eps_m && fabsf(delta_y_m) <= eps_m)
    {
        path_follow_reset_soft_start_state();
        pf_path = NULL;
        pf_steps = 0;
        pf_idx = 0;
        pf_active = 0;
        return;
    }

    // 发车偏移路径使用固定分辨率（1cm）
    pf_grid_m = PRESTART_OFFSET_RESOLUTION_M;

    // 第 0 个点：起点
    g_prestart_offset_path[0].row = (int)lroundf(start_x_m / pf_grid_m);
    g_prestart_offset_path[0].col = (int)lroundf(start_y_m / pf_grid_m);

    // 如果 X 方向有位移，加入中间点
    if (fabsf(delta_x_m) > eps_m)
    {
        g_prestart_offset_path[steps].row = (int)lroundf(mid_x_m / pf_grid_m);
        g_prestart_offset_path[steps].col = (int)lroundf(mid_y_m / pf_grid_m);
        steps++;
    }

    // 如果 Y 方向有位移，加入终点
    if (fabsf(delta_y_m) > eps_m)
    {
        g_prestart_offset_path[steps].row = (int)lroundf(end_x_m / pf_grid_m);
        g_prestart_offset_path[steps].col = (int)lroundf(end_y_m / pf_grid_m);
        steps++;
    }

    // 把偏移点序列作为当前路径下发
    path_follow_set_path(g_prestart_offset_path, steps);
}
// 用四轮编码器估计车体位姿
static void update_odometry(float yaw_deg)//航位推算
{
    // 由四轮编码器反解车体速度，再积分得到世界坐标位姿
    if (pf_pulses_per_meter <= 0.0f)
    {
        return;
    }

    float count_to_mps = ((float)PID_RATE) / pf_pulses_per_meter;
    float actual_ul = (float)up_L_all * count_to_mps;
    float actual_ur = (float)up_R_all * count_to_mps;
    float actual_dl = (float)down_L_all * count_to_mps;
    float actual_dr = (float)down_R_all * count_to_mps;

    float vx_body = 0.25f * (actual_ul + actual_ur + actual_dl + actual_dr);
    float vy_body = 0.25f * (-actual_ul + actual_ur + actual_dl - actual_dr);
    float omega_body = (-actual_ul + actual_ur - actual_dl + actual_dr) / (2*D_X + 2*D_Y);
    // if (rx_plus_ry_cali != 0.0f)
    // {
    //     omega_body /= rx_plus_ry_cali;
    // }

    float dt = 1.0f / (float)PID_RATE;
    float yaw_rad = yaw_deg * ((float)M_PI / 180.0f);

    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    float vx_world = vx_body * cos_yaw - vy_body * sin_yaw;
    float vy_world = vx_body * sin_yaw + vy_body * cos_yaw;

    // 位置积分更新
    actual_x_position += vx_world * dt;//变量就是当前位置坐标
    actual_y_position += vy_world * dt;
    pf_yaw_deg = yaw_deg;
}

// 路径跟随输出车体系三轴速度
void path_follow_update(float yaw_deg, path_follow_output_t *out)//传递路径，更新需要的位置环输出的目标速度
{
    // 1) 先给输出清零，保证异常路径时不会保留旧命令
    if (out)
    {
        out->active = 0;
        out->reached = 0;
        out->vx_cmd = 0;
        out->vy_cmd = 0;
        out->omega_cmd = 0;
        out->target_idx = pf_idx;
    }

    if (!pf_active || NULL == pf_path || 0 == pf_steps)
    {
        // 没有有效路径就直接返回
        return;
    }

    update_odometry(yaw_deg);
    // path_follow_update_pid_limits();

    Point target = pf_path[pf_idx];
    float target_x = target.row * pf_grid_m;
    float target_y = target.col * pf_grid_m;

    float dx = target_x - actual_x_position;//当前目标位置和实际位置的差
    float dx_fabs = fabsf(dx);
    float dy = target_y - actual_y_position;
    float dy_fabs = fabsf(dy);
    float dist = sqrtf(dx * dx + dy * dy);

    if (pf_idx > 0 && pf_active == 1)
    {
        if (pf_path[pf_idx].row - pf_path[pf_idx-1].row == 0)
        {
            car_direction = 2; // Y 方向
        }
        else if (pf_path[pf_idx].col - pf_path[pf_idx-1].col == 0)
        {
            car_direction = 1; // X 方向
        }
        else
        {
            car_direction = 3; // XY 方向
        }
    }
    else
    {
        car_direction = 0; // 停止
    }

    if (dist < pf_pos_tol_m)
    {
        // 到达当前点：切下一个点；如果没有下一个点则结束
        if (pf_idx + 1 < pf_steps)
        {
            pf_idx++;
            target = pf_path[pf_idx];
            target_x = target.row * pf_grid_m;
            target_y = target.col * pf_grid_m;
            dx = target_x - actual_x_position;
            dy = target_y - actual_y_position;
            dx_fabs = fabsf(dx);
            dy_fabs = fabsf(dy);
            dist = sqrtf(dx * dx + dy * dy);
        }
        else
        {
            pf_active = 0;
            path_follow_reset_soft_start_state();
            if (out)
            {
                out->reached = 1;
            }
            return;
        }
    }

    // float desired_heading_rad = atan2f(dy, dx);
    // float desired_heading_deg = desired_heading_rad * (180.0f / (float)M_PI);
    float yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    float v_world_x_target_cmd = 0.0f;
    float v_world_y_target_cmd = 0.0f;
    float v_world_x = 0.0f;
    float v_world_y = 0.0f;

    if (car_direction == 1)
    {
        // X 方向跟随：Y 方向只做保持
        if (dx_fabs > DX_FAST_DISTANCE)
        {
           v_world_x_target_cmd = fsignal(dx) * f_my_min(CAR_DRIVER_MAX_SPEED,100*sqrtf(2*CAR_DRIVER_MAX_ACCELERATE*dx_fabs)) + 0.1*PID_Location_Calculate(&pid_world_x, actual_x_position*100, target_x*100);
        }
        else if (dx_fabs > DX_SLOW_DISTANCE && dx_fabs <= DX_FAST_DISTANCE)
        {
           v_world_x_target_cmd = fsignal(dx) * f_my_max(PID_Location_Calculate(&pid_world_x, actual_x_position*100, target_x*100), CAR_DRIVER_MIN_SPEED);    
        }
        else if (dx_fabs <= DX_SLOW_DISTANCE)
        {
           v_world_x_target_cmd = PID_Location_Calculate(&pid_stay, actual_x_position*100, target_x*100) + fsignal(dx) * 90*28*(-1*(dx_fabs - 0.1f)*(dx_fabs - 0.1f)+0.01f);
        }

        v_world_y_target_cmd = PID_Location_Calculate(&pid_stay, actual_y_position*100, target_y*100);
       v_world_x_target_cmd = path_follow_apply_soft_start(v_world_x_target_cmd, dx_fabs, &g_start_ramp_cmps, &g_start_ramp_sign);
       v_world_x_target_cmd = path_follow_apply_quick_stop_limit(v_world_x_target_cmd, dx_fabs);
        path_follow_reset_soft_start();
    }
    else if (car_direction == 2)
    {
        // Y 方向跟随：X 方向只做保持
        if (dy_fabs > DY_FAST_DISTANCE)
        {
            v_world_y_target_cmd = fsignal(dy) * f_my_min(CAR_DRIVER_MAX_SPEED,100*sqrtf(2*CAR_DRIVER_MAX_ACCELERATE*dy_fabs)) + 0.1*PID_Location_Calculate(&pid_world_y, actual_y_position*100, target_y*100);
        }
        else if (dy_fabs > DY_SLOW_DISTANCE && dy_fabs <= DY_FAST_DISTANCE)
        {
            v_world_y_target_cmd = fsignal(dy) * f_my_max(PID_Location_Calculate(&pid_world_y, actual_y_position*100, target_y*100), CAR_DRIVER_MIN_SPEED);    
        }
        else if (dy_fabs <= DY_SLOW_DISTANCE)
        {
            v_world_y_target_cmd = PID_Location_Calculate(&pid_stay, actual_y_position*100, target_y*100)+ fsignal(dy) * 90*28*(-1*(dy_fabs - 0.1f)*(dy_fabs - 0.1f)+0.01f);
        }

       v_world_x_target_cmd = PID_Location_Calculate(&pid_stay, actual_x_position*100, target_x*100);
        v_world_y_target_cmd = path_follow_apply_soft_start(v_world_y_target_cmd, dy_fabs, &g_start_ramp_cmps, &g_start_ramp_sign);
        v_world_y_target_cmd = path_follow_apply_quick_stop_limit(v_world_y_target_cmd, dy_fabs);
        path_follow_reset_soft_start();
    }
    else
    {
        // 非轴向路径或异常情况，先停
       v_world_x_target_cmd = 0;
        v_world_y_target_cmd = 0;
        path_follow_reset_soft_start_state();
    }
    
    v_world_x =v_world_x_target_cmd;
    v_world_y = v_world_y_target_cmd;


    // if (v_world_x > 0.0f && v_world_x < CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_x = CAR_DRIVER_MIN_SPEED;
    // }
    // else if (v_world_x < 0.0f && v_world_x > -CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_x = -CAR_DRIVER_MIN_SPEED;
    // }
    
    // if (v_world_y > 0.0f && v_world_y < CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_y = CAR_DRIVER_MIN_SPEED;
    // }
    // else if (v_world_y < 0.0f && v_world_y > -CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_y = -CAR_DRIVER_MIN_SPEED;
    // }

    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);
    float v_body_x = v_world_x * cos_yaw + v_world_y * sin_yaw;
    float v_body_y = -v_world_x * sin_yaw + v_world_y * cos_yaw;

    // 航向保持到 0 度
    float omega_cmd = PID_Location_Calculate(&pid_yaw, yaw_deg, 0);
    // float omega_accel_cmd = PID_Location_Calculate(&pid_accel_yaw, omega_cmd, 0);
    omega_cmd = clamp_sym(omega_cmd, pf_max_w_rad);

    if (out)
    {
        out->vx_cmd = (v_body_x);   // 速度输出：cm/s
        out->vy_cmd = (v_body_y);
        out->omega_cmd = omega_cmd*(M_PI / 180.0f);               // 角速度输出：rad/s
        out->active = 1;
        out->target_idx = pf_idx;
    }
}

// 路径跟随输出车体系三轴速度
void path_follow_update_test(float yaw_deg, path_follow_output_t *out)
{
    // 测试版本：只走定点保持 PID，不走分段速度策略
    if (out)
    {
        out->active = 0;
        out->reached = 0;
        out->vx_cmd = 0;
        out->vy_cmd = 0;
        out->omega_cmd = 0;
        out->target_idx = pf_idx;
    }

    if (!pf_active || NULL == pf_path || 0 == pf_steps)
    {
        return;
    }

    update_odometry(yaw_deg);
    // path_follow_update_pid_limits();

    Point target = pf_path[pf_idx];
    float target_x = target.row * pf_grid_m;
    float target_y = target.col * pf_grid_m;

    float dx = target_x - actual_x_position;
    float dx_fabs = fabsf(dx);
    float dy = target_y - actual_y_position;
    float dy_fabs = fabsf(dy);
    float dist = sqrtf(dx * dx + dy * dy);


    // float desired_heading_rad = atan2f(dy, dx);
    // float desired_heading_deg = desired_heading_rad * (180.0f / (float)M_PI);
    float yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    int v_world_x_target_cmd = 0;
    int v_world_y_target_cmd = 0;
    int v_world_x = 0;
    int v_world_y = 0;


    v_world_y_target_cmd = PID_Location_Calculate(&pid_stay, actual_y_position*100, target_y*100);//世界坐标下的根据位置环算出的需要的速度
   v_world_x_target_cmd = PID_Location_Calculate(&pid_stay, actual_x_position*100, target_x*100);

    // if (abs(v_world_x_target_cmd) < 8 && abs(v_world_x_target_cmd) > 3)
    // {
    //     v_world_x = (float)v_world_x_target_cmd + fsignal(v_world_x_target_cmd)*1.0f;
    // }
    // else
    // {
        v_world_x = (float)v_world_x_target_cmd;//实际调用前一个浮点数
    // }

    // if (abs(v_world_y_target_cmd) < 8 && abs(v_world_y_target_cmd) > 3)
    // {
    //     v_world_y = (float)v_world_y_target_cmd + fsignal(v_world_y_target_cmd)*1.0f;
    // }
    // else
    // {
        v_world_y = (float)v_world_y_target_cmd;
    // }

    // if (v_world_x > 0.0f && v_world_x < CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_x = CAR_DRIVER_MIN_SPEED;
    // }
    // else if (v_world_x < 0.0f && v_world_x > -CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_x = -CAR_DRIVER_MIN_SPEED;
    // }
    
    // if (v_world_y > 0.0f && v_world_y < CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_y = CAR_DRIVER_MIN_SPEED;
    // }
    // else if (v_world_y < 0.0f && v_world_y > -CAR_DRIVER_MIN_SPEED)
    // {
    //     v_world_y = -CAR_DRIVER_MIN_SPEED;
    // }

    float cos_yaw = cosf(yaw_rad);//因为yaw始终理论控制在0，所以没啥用吧。。。？
    float sin_yaw = sinf(yaw_rad);
    float v_body_x = v_world_x * cos_yaw + v_world_y * sin_yaw;
    float v_body_y = -v_world_x * sin_yaw + v_world_y * cos_yaw;
    // float v_body_x = 0;
    // float v_body_y = 0;

    float omega_cmd = PID_Location_Calculate(&pid_yaw, yaw_deg, 0);//最后一个就是目标yaw一直为0
    omega_cmd = clamp_sym(omega_cmd, pf_max_w_rad);//限幅的，没啥用

    if (out)
    {
        out->vx_cmd = (v_body_x);   // 速度输出：cm/s
        out->vy_cmd = (v_body_y);
        out->omega_cmd = omega_cmd*(M_PI / 180.0f);               // 角速度输出：rad/s
        out->active = 1;
        out->target_idx = pf_idx;
    }
}

void distance_speed_strategy(void)//把整体的目标速度下放到各个轮子的速度环控制
{
    // 位置环输出 -> 三轴速度 -> 逆运动学 -> 四轮目标编码器速度
    path_follow_output_t pf = {0};

    path_follow_update(eulerAngle.yaw, &pf);
    if (pf.active)//标志位
    {
        speed_three_array[0] = pf.vx_cmd;//传入目标速度
        speed_three_array[1] = pf.vy_cmd;
        speed_three_array[2] = pf.omega_cmd;
    }
    else
    {
        speed_three_array[0] = 0;
        speed_three_array[1] = 0;
        speed_three_array[2] = 0;
    }

    Kinematics_Inverse(speed_three_array, speed_encoder);//第一个数组是输入根据位置环给出的目标速度，第二个是输出的需要各个轮子需要的目标速度具体控制速度环
}

// 获取当前位姿与目标信息
void path_follow_get_status(path_follow_status_t *status)
{
    // 读出当前里程计与目标点状态，供主循环/菜单显示
    if (NULL == status)
    {
        return;
    }

    status->x_m = actual_x_position;//把当前世界坐标赋值为编码器算出的坐标
    status->y_m = actual_y_position;
    status->yaw_deg = pf_yaw_deg;
    status->active = pf_active;
    status->reached = (pf_active == 0);
    status->target_idx = pf_idx;

    if (pf_path && pf_idx < pf_steps)
    {
        status->target_x_m = pf_path[pf_idx].row * pf_grid_m;
        status->target_y_m = pf_path[pf_idx].col * pf_grid_m;
    }
    else
    {
        status->target_x_m = 0.0f;
        status->target_y_m = 0.0f;
    }
}

// 在 IPS200 显示位姿与目标点
void path_follow_draw_status(void)
{
    // 在屏幕上实时显示路径跟随状态
    path_follow_status_t st = {0};
    path_follow_get_status(&st);

    // ips200_show_string(x, y, "Pose x y yaw:");
    ips200_show_string(0,112,"st_x_m");
    ips200_show_float(70, 112, st.x_m, 2, 4);
    ips200_show_string(0,128,"st_y_m");
    ips200_show_float(70, 128, st.y_m, 2, 4);
    ips200_show_string(0,144,"st_yaw");
    ips200_show_float(70, 144, st.yaw_deg, 3, 2);

    // ips200_show_string(x, y + 16, "Target idx/x/y:");
    ips200_show_string(0,160,"target_idx");
    ips200_show_uint(90, 160, st.target_idx, 3);
    ips200_show_string(0,176,"target_x_m");
    ips200_show_float(90, 176, st.target_x_m, 2, 3);
    ips200_show_string(0,192,"target_y_m");
    ips200_show_float(90, 192, st.target_y_m, 2, 3);

}

// 航向定义：前方 0 度，左正右负
float path_follow_heading_deg(Point from, Point to)//这个函数目前没用，因为都是走直线，不需要转角。本来的作用是计算方向角的
{
    // 计算 from->to 的几何航向角（度）
    float dx = (float)(to.row - from.row);
    float dy = (float)(to.col - from.col);
    if (dx == 0.0f && dy == 0.0f)
    {
        return 0.0f;
    }
    float angle_rad = atan2f(dy, dx);
    return angle_rad * (180.0f / (float)M_PI);
}

// 提取路径拐点
size_t path_follow_extract_corners(const Point *path, size_t path_steps, 
                                   Point *corner_buffer, size_t corner_capacity)//路径指针，路径点数量，拐点输出缓冲区，拐点缓冲区容量
{
    // 参数无效直接失败
    if (NULL == path || 0 == path_steps || NULL == corner_buffer || 0 == corner_capacity)
    {
        return 0;
    }

    // 仅一个点时直接返回
    if (path_steps == 1)
    {
        // 缓冲区够：复制唯一点并返回
        if (corner_capacity >= 1)
        {
            corner_buffer[0] = path[0];
            return 1;
        }
        // 缓冲区不够：失败
        return 0;
    }

    // 当前已写入拐点数量
    size_t corner_count = 0;

    // 起点一定是拐点
    if (corner_count >= corner_capacity)
    {
        return 0;  // 缓冲区不足
    }
    corner_buffer[corner_count++] = path[0];//把起点写入拐点缓冲区

    // 检查中间点是否拐弯
    for (size_t i = 1; i < path_steps - 1; i++)
    {
        // 上一段方向向量
        int dx1 = path[i].col - path[i - 1].col;
        int dy1 = path[i].row - path[i - 1].row;

        // 下一段方向向量
        int dx2 = path[i + 1].col - path[i].col;
        int dy2 = path[i + 1].row - path[i].row;

        // 方向变化则记为拐点
        if (dx1 != dx2 || dy1 != dy2)
        {
            // 每次写入前先检查容量
            if (corner_count >= corner_capacity)
            {
                return 0;  // 缓冲区不足
            }
            // 写入当前拐点
            corner_buffer[corner_count++] = path[i];
        }
    }

    // 终点一定是拐点
    if (corner_count >= corner_capacity)
    {
        return 0;  // 缓冲区不足
    }
    // 终点写入
    corner_buffer[corner_count++] = path[path_steps - 1];

    // 返回拐点总数
    return corner_count;
}

