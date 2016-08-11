#include <stm32f4xx.h>
#include "main.h"

#define MOUSE_SENSITIVITY (10)

volatile extern RC_Ctl_t RC_Ctl;
uint8_t Remote_On = 0;
// global control information req
volatile int16_t drive;
volatile int16_t strafe;
volatile int16_t rotate;
volatile extern int16_t pitch_Position;
volatile extern int16_t yaw_Position;
volatile int16_t friction_motor_state;
volatile int16_t feeder_motor_state;
volatile int manual_Control_Turret = 0;


// volatile extern int16_t pitch_Velocity;
// volatile extern int16_t yaw_Velocity;

// for velocity controlling pitch and yaw with remote
// volatile extern int16_t remote_pitch_change;
// volatile extern int16_t remote_yaw_change;
volatile extern arduino_data data_usart_3;

#define ROBOT_STATE_MANUAL (0)
#define ROBOT_STATE_SEMI_AUTO (1)
#define ROBOT_STATE_SPECIAL (2)



uint8_t remote_on = 0;
uint8_t robot_state = ROBOT_STATE_MANUAL;

// this is run by tim2
void Remote_Control() {

    if (RC_Ctl.rc.ch2 < RC_CH_VALUE_MIN || RC_Ctl.rc.ch3 < RC_CH_VALUE_MIN) {
        remote_on = 0;
    } else {
        remote_on = 1;
    }

    if (remote_on) {
        // (1) read switch positions and store into state
        // s1 for robot state
        switch (RC_Ctl.rc.s1) {
            case RC_SW_UP:
                robot_state = ROBOT_STATE_MANUAL;
                break;
            case RC_SW_MID:
                robot_state = ROBOT_STATE_SEMI_AUTO;
                break;
            case RC_SW_DOWN:
                robot_state = ROBOT_STATE_SPECIAL;
                break;
        }

        // (2) set global control info requests
        switch (robot_state) {
            case ROBOT_STATE_MANUAL:
                if (ROBOT_SERIAL_NUMBER != HERO_ROBOT_CANNON_7) {
                    // driving
                    if (ROBOT_SERIAL_NUMBER == HERO_ROBOT_TURRET_8) {
                        drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
                        strafe = 0;
                        rotate = (RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET);
                    } else {
                        drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
                        strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
                        // rotate = (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET) + RC_Ctl.mouse.x * MOUSE_SENSITIVITY;
                        rotate = RC_Ctl.mouse.x * MOUSE_SENSITIVITY;
                    }

                    // shooting
                    friction_motor_state = (RC_Ctl.rc.s2 == RC_SW_MID || RC_Ctl.rc.s2 == RC_SW_DOWN);
                    feeder_motor_state = (RC_Ctl.rc.s2 == RC_SW_DOWN || RC_Ctl.mouse.press_l);

                    // aiming (untested) todo: limit these two
                    pitch_Position += (RC_Ctl.rc.ch1 - RC_CH_VALUE_OFFSET) / 100;
                    yaw_Position += (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET) / 100 + RC_Ctl.mouse.y * MOUSE_SENSITIVITY;
                }
                break;
            case ROBOT_STATE_SEMI_AUTO:
                if (ROBOT_SERIAL_NUMBER != HERO_ROBOT_CANNON_7) {
                    // driving
                    drive = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
                    strafe = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
                    rotate = RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET;

                    // shooting
                    friction_motor_state = data_usart_3.packet.friction_motor_state;
                    feeder_motor_state = data_usart_3.packet.feeder_motor_state;

                    // aiming
                    pitch_Position = data_usart_3.packet.pitch_req;
                    yaw_Position = data_usart_3.packet.yaw_req;
                }
                break;
            case ROBOT_STATE_SPECIAL:
                if (ROBOT_SERIAL_NUMBER != HERO_ROBOT_CANNON_7) {
                    // driving
                    drive = data_usart_3.packet.drive_req;
                    strafe = data_usart_3.packet.strafe_req;
                    rotate = data_usart_3.packet.rotate_req;

                    // shooting
                    friction_motor_state = data_usart_3.packet.friction_motor_state;
                    feeder_motor_state = data_usart_3.packet.feeder_motor_state;

                    // aiming
                    pitch_Position = data_usart_3.packet.pitch_req;
                    yaw_Position = data_usart_3.packet.yaw_req;
                } else {
                    // shooting is controlled by the arduino

                    // aiming with received data from arduino
                    pitch_Position += (RC_Ctl.rc.ch1 - RC_CH_VALUE_OFFSET) / 100;
                    yaw_Position += (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET) / 100;
                }
                break;
        }
    }
    if (data_usart_3.packet.js_real_chassis_out_power > 10) {
        LED2_ON();
    } else {
        LED2_OFF();
    }
}

//return the state of the remote 0:no action 1:action
uint8_t IsRemoteBeingAction(void)
{
	return (fabs(drive)>=10 || fabs(strafe)>=10 || fabs((RC_Ctl.rc.ch1 - RC_CH_VALUE_OFFSET))>=10 || fabs(RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET)>=10);
}

static volatile Shoot_State_e shootState = NOSHOOTING;

Shoot_State_e GetShootState()
{
  	if (feeder_motor_state == 0) {
  	    shootState = NOSHOOTING;
  	}
    else {
        shootState = SHOOTING;
    }
    return shootState;
}

// division rounding
// for smoothening remote control of pitch and yaw
int round_div(int dividend, float divisor)
{
    return (int) ((dividend + (divisor / 2)) / divisor);
}
