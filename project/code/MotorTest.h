#ifndef __MOTOR_TEST_H_
#define __MOTOR_TEST_H_

/*
 * Set to 1 for new-board motor bring-up.
 * This bypasses the normal camera/path/IMU/menu workflow in main().
 * Set back to 0 before running the normal competition program.
 */
#define MOTOR_BOARD_TEST_ENABLE 0

void motor_board_test_run(void);

#endif
