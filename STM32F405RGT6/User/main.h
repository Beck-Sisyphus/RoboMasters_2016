#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx.h"
#include "mpu6050_i2c.h"
#include "mpu6050_interrupt.h"
#include "mpu6050_driver.h"
#include "mpu6050_process.h"
//#include "hmc5883_driver.h"

#include "bsp.h"
#include "module_rs232.h"
#include "HAL_HMC5883L.h"
#include "HMC5883L.h"
#include "CanBusTask.h"
#include "remote_control.h"
#include "ControlTask.h"


/*  Define the robot as
    0 Blue sample robot
    1 Red sample robot
    2~5 Soldiers we manufacture in 2016
    6 Base robot
    7 Hero robot */
#define ROBOT_SERIAL_NUMBER 5

// For Blue motor and all 2016 soldier robots
#define BLUE_SAMPLE_ROBOT_0   0
#define RED_SAMPLE_ROBOT_1    1
#define SOLDIER_2           2
#define SOLDIER_3           3
#define SOLDIER_4           4
#define SOLDIER_5           5
#define BASE_ROBOT_6       		6
#define HERO_ROBOT_CANNON_7   7
#define HERO_ROBOT_TURRET_8   8

#endif
