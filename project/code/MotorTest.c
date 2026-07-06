#include "zf_common_headfile.h"
#include "MotorTest.h"
#include "Motor.h"
#include "BlueSerial.h"
#include "PID_config.h"

#define MOTOR_TEST_PWM                 1800
#define MOTOR_TEST_STAGE_MS            2500U
#define MOTOR_TEST_STOP_MS             800U
#define MOTOR_TEST_SAMPLE_MS           10U
#define MOTOR_TEST_REPORT_MS           100U
#define MOTOR_TEST_BANNER_REPEAT_MS    1000U

typedef struct
{
    const char *name;
    int ul_pwm;
    int ur_pwm;
    int dl_pwm;
    int dr_pwm;
    uint32 duration_ms;
} motor_test_step_t;

static const motor_test_step_t g_motor_test_steps[] =
{
    {"UL_FWD",       MOTOR_TEST_PWM, 0, 0, 0, MOTOR_TEST_STAGE_MS},
    {"UL_REV",      -MOTOR_TEST_PWM, 0, 0, 0, MOTOR_TEST_STAGE_MS},
    {"UR_FWD",       0, MOTOR_TEST_PWM, 0, 0, MOTOR_TEST_STAGE_MS},
    {"UR_REV",       0, -MOTOR_TEST_PWM, 0, 0, MOTOR_TEST_STAGE_MS},
    {"DL_FWD",       0, 0, MOTOR_TEST_PWM, 0, MOTOR_TEST_STAGE_MS},
    {"DL_REV",       0, 0, -MOTOR_TEST_PWM, 0, MOTOR_TEST_STAGE_MS},
    {"DR_FWD",       0, 0, 0, MOTOR_TEST_PWM, MOTOR_TEST_STAGE_MS},
    {"DR_REV",       0, 0, 0, -MOTOR_TEST_PWM, MOTOR_TEST_STAGE_MS},
    {"ALL_FORWARD",  MOTOR_TEST_PWM, MOTOR_TEST_PWM, MOTOR_TEST_PWM, MOTOR_TEST_PWM, MOTOR_TEST_STAGE_MS},
    {"ALL_BACK",    -MOTOR_TEST_PWM, -MOTOR_TEST_PWM, -MOTOR_TEST_PWM, -MOTOR_TEST_PWM, MOTOR_TEST_STAGE_MS},
    {"STRAFE_LEFT", -MOTOR_TEST_PWM, MOTOR_TEST_PWM, MOTOR_TEST_PWM, -MOTOR_TEST_PWM, MOTOR_TEST_STAGE_MS},
    {"STRAFE_RIGHT", MOTOR_TEST_PWM, -MOTOR_TEST_PWM, -MOTOR_TEST_PWM, MOTOR_TEST_PWM, MOTOR_TEST_STAGE_MS},
    {"ROTATE_CW",   -MOTOR_TEST_PWM, MOTOR_TEST_PWM, -MOTOR_TEST_PWM, MOTOR_TEST_PWM, MOTOR_TEST_STAGE_MS},
    {"ROTATE_CCW",   MOTOR_TEST_PWM, -MOTOR_TEST_PWM, MOTOR_TEST_PWM, -MOTOR_TEST_PWM, MOTOR_TEST_STAGE_MS},
};

static void motor_test_clear_drive_state(void)
{
    motor_pwm(0, 0, 0, 0);
    PID_Clear(&ULpid);
    PID_Clear(&URpid);
    PID_Clear(&DLpid);
    PID_Clear(&DRpid);
    encoder_get();
}

static void motor_test_print_header(void)
{
    BlueSerial_Printf("\r\nMOTOR_TEST START pwm=%d stage_ms=%u\r\n",
                      MOTOR_TEST_PWM,
                      MOTOR_TEST_STAGE_MS);
    BlueSerial_Printf("order: UL UR DL DR, speed fields are filtered encoder counts per 10ms sample\r\n");
}

static void motor_test_report(const char *step_name, uint32 elapsed_ms)
{
    BlueSerial_Printf("MT %-12s t=%lu pwm[%d,%d,%d,%d] enc[%d,%d,%d,%d]\r\n",
                      step_name,
                      (unsigned long)elapsed_ms,
                      speed_encoder[0],
                      speed_encoder[1],
                      speed_encoder[2],
                      speed_encoder[3],
                      up_L_all,
                      up_R_all,
                      down_L_all,
                      down_R_all);
}

static void motor_test_apply_step(const motor_test_step_t *step)
{
    if (step == NULL)
    {
        return;
    }

    speed_encoder[0] = step->ul_pwm;
    speed_encoder[1] = step->ur_pwm;
    speed_encoder[2] = step->dl_pwm;
    speed_encoder[3] = step->dr_pwm;
    motor_pwm(step->ul_pwm, step->ur_pwm, step->dl_pwm, step->dr_pwm);
}

static void motor_test_run_step(const motor_test_step_t *step)
{
    uint32 elapsed_ms = 0U;

    if (step == NULL)
    {
        return;
    }

    motor_test_apply_step(step);
    BlueSerial_Printf("\r\nSTEP %s pwm[%d,%d,%d,%d]\r\n",
                      step->name,
                      step->ul_pwm,
                      step->ur_pwm,
                      step->dl_pwm,
                      step->dr_pwm);

    while (elapsed_ms < step->duration_ms)
    {
        system_delay_ms(MOTOR_TEST_SAMPLE_MS);
        elapsed_ms += MOTOR_TEST_SAMPLE_MS;
        encoder_get();
        if ((elapsed_ms % MOTOR_TEST_REPORT_MS) == 0U)
        {
            motor_test_report(step->name, elapsed_ms);
        }
    }

    motor_test_clear_drive_state();
    system_delay_ms(MOTOR_TEST_STOP_MS);
    encoder_get();
    motor_test_report("STOP", 0U);
}

void motor_board_test_run(void)
{
    uint32 i = 0U;
    uint32 idle_banner_ms = 0U;

    motor_init();
    encoder_init();
    Blue_Serial_Init();

    PID_Init(&ULpid, &ULPidInitStruct);
    PID_Init(&URpid, &URPidInitStruct);
    PID_Init(&DLpid, &DLPidInitStruct);
    PID_Init(&DRpid, &DRPidInitStruct);
    Kinematics_Init();

    motor_test_clear_drive_state();
    system_delay_ms(500U);
    motor_test_print_header();

    while (1)
    {
        for (i = 0U; i < (sizeof(g_motor_test_steps) / sizeof(g_motor_test_steps[0])); i++)
        {
            motor_test_run_step(&g_motor_test_steps[i]);
        }

        motor_test_clear_drive_state();
        BlueSerial_Printf("\r\nMOTOR_TEST CYCLE_DONE, all motors stopped\r\n");

        idle_banner_ms = 0U;
        while (idle_banner_ms < 5000U)
        {
            system_delay_ms(MOTOR_TEST_REPORT_MS);
            idle_banner_ms += MOTOR_TEST_REPORT_MS;
            encoder_get();
            if ((idle_banner_ms % MOTOR_TEST_BANNER_REPEAT_MS) == 0U)
            {
                motor_test_report("IDLE", idle_banner_ms);
            }
        }
    }
}
