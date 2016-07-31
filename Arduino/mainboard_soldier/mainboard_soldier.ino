#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "dji_js.h"

#define TX1_TPZ_PACKET_SIZE (32)
#define STORAGE_DATA_SIZE (TX1_TPZ_PACKET_SIZE / 2)
#define TPZ_OUT_HEADER (0xFA)
#define TX1_OUT_HEADER (0xAA)
#define HTPZ_OUT_HEADER (0xFA)

// pin definitions
#define ARM_UP_PIN (2) // hero only
#define ARM_DN_PIN (3) // hero only
#define FAN_PIN (4) // hero only
#define STEPPER_MOTOR_PIN (5) // hero only
#define LED_PIN (13)
#define FEEDER_MOTOR_PIN (37)

#define MPU_ENABLE false // set to true to enable mpu
#define DEBUG false // set to true to enable debug prints, will disable arduino->tx1 comms
#define HERO_ARDUINO false // set to true if this arduino goes on hero robot

#define LED_ON() digitalWrite(LED_PIN, HIGH)
#define LED_OFF() digitalWrite(LED_PIN, LOW)
#define L_BYTE(b) (b >> 8) & 255
#define H_BYTE(b) (b & 255)
#define MAKE_INT16(a, b) (((int16_t) a << 8)) | (b & 255)

/**** rc switch definition ****/
#define RC_SW_UP                        ((uint16_t)1)
#define RC_SW_MID                       ((uint16_t)3)
#define RC_SW_DOWN                      ((uint16_t)2)

/**** pc key definition ****/
#define KEY_PRESSED_OFFSET_W            ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)0x01<<7)

/******** kalman ********/
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances

const uint8_t MPU6050 = 0x68; // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69
const uint8_t HMC5883L = 0x1E; // Address of magnetometer

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;
int16_t tempRaw;

double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

#define MAG0MAX 603
#define MAG0MIN -578

#define MAG1MAX 542
#define MAG1MIN -701

#define MAG2MAX 547
#define MAG2MIN -556

float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
double magGain[3];

#define KAL_CONST_X (100)
#define KAL_CONST_Y (100)
#define KAL_CONST_Z (100)

int16_t kal_int_x;
int16_t kal_int_y;
int16_t kal_int_z;
/************************/

/********dji_js********/

// data structurs which store the information obtained from judgement system
GENERAL_INFO *general_info;
HEALTH_DATA *health_data;
WEAPON_DATA *weapon_data;

// The byte read from judgement system
unsigned char js_rx_byte;

/**********************/

// com buffers
char tx1_in_buf[TX1_TPZ_PACKET_SIZE]; // incoming serial buffer
char tpz_out_buf[TX1_TPZ_PACKET_SIZE]; // outgoing serial buffer
char htpz_out_buf[32]; // outgoing serial buffer for hero trapezoid
char tpz_in_buf[32];
char tx1_out_buf[32];

// data buffers
int16_t tx1_tpz_data[STORAGE_DATA_SIZE]; // data in int

// control variables
int16_t feeder_motor_state_req;

// output js data
uint16_t js_real_chassis_out_power; // power in watts
uint16_t js_remain_life_value;
uint8_t js_big_rune_0_status;
uint8_t js_big_rune_1_status;

unsigned long tpz_timer_prev_time = 0;
const long tpz_timer_period = 20; // time between transmitting packets
unsigned long tx1_timer_prev_time = 0;
const long tx1_timer_period = 100; // 10hz

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(FEEDER_MOTOR_PIN, OUTPUT);
    Serial.begin(9600); // tx1
    Serial1.begin(115200); // tpz (trapezoid)
#if HERO_ARDUINO
    // init hero interface
    Serial2.begin(115200); // htpz (hero trapezoid)
    pinMode(ARM_UP_PIN, OUTPUT);
    pinMode(ARM_DN_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(STEPPER_MOTOR_PIN, OUTPUT);
#endif

#if MPU_ENABLE
    // INIT mpu
    mpu_kalman_init();
#endif

    // INIT judgement system
    Serial3.begin(115200);
    general_info = (GENERAL_INFO *) malloc(sizeof(general_info));
    general_info->gps_data = (GPS_DATA *) malloc(sizeof(GPS_DATA));
    health_data = (HEALTH_DATA *) malloc(sizeof(health_data));
    weapon_data = (WEAPON_DATA *) malloc(sizeof(weapon_data));
}

void loop() {
    // (1) process mpu kalman
#if MPU_ENABLE
    mpu_kalman_process();
#endif

    // (2) process packet from tx1
    if (Serial.available() > 0) {//TX1_TPZ_PACKET_SIZE - 1) {
        if (Serial.peek() != 0xF9) {
            Serial.read();
        } else {
            // receive from tx1 into buffer
            Serial.readBytes((char*) tx1_in_buf, TX1_TPZ_PACKET_SIZE);

            // cleanup buffer
            for (int i = 0; i < TX1_TPZ_PACKET_SIZE; i++) {
                tx1_in_buf[i] = (tx1_in_buf[i] & 255);
            }

            // set into int16 buffer according to protocol spec
            for (int i = 0; i < STORAGE_DATA_SIZE; i++) {
                tx1_tpz_data[i * 2] = ((int16_t) tx1_in_buf[i * 2] << 8) | (tx1_in_buf[i * 2 + 1] & 255);
            }
        }
    }

    // (2.5) process packet from tpz
    if (Serial1.available() > 0) {
        if (Serial1.peek() != 0xCE) {
            Serial1.read();
        } else {
            // receive from tpz into buffer
            Serial1.readBytes((char*) tpz_in_buf, 32);

            // cleanup buffer
            for (int i = 0; i < 32; i++) {
                tpz_in_buf[i] = (tpz_in_buf[i] & 255);
            }

            // set control variables
            feeder_motor_state_req = tpz_in_buf[2];

#if HERO_ARDUINO
            /********(2.5.1) process received data********/
            int16_t key_req = MAKE_INT16(tpz_in_buf[28], tpz_in_buf[29]);

            // arm control
            if (key_req & KEY_PRESSED_OFFSET_W) { // arm up movement
                digitalWrite(ARM_UP_PIN, HIGH);
                digitalWrite(ARM_DN_PIN, LOW);
            } else if (key_req & KEY_PRESSED_OFFSET_S) { // arm dn movement
                digitalWrite(ARM_UP_PIN, LOW);
                digitalWrite(ARM_DN_PIN, HIGH);
            } else { // no movement
                digitalWrite(ARM_UP_PIN, LOW);
                digitalWrite(ARM_DN_PIN, LOW);
            }

            // fan control
            if (key_req & KEY_PRESSED_OFFSET_D) { // fan on
                digitalWrite(FAN_PIN, HIGH);
            } else { // fan off
                digitalWrite(FAN_PIN, LOW);
            }

            // stepper motor control
            if (tpz_in_buf[18] == RC_SW_DOWN) { // special function on hero controls cannon
                if (tpz_in_buf[19] == RC_SW_DOWN || tpz_in_buf[26]) {
                    digitalWrite(STEPPER_MOTOR_PIN, HIGH);
                } else {
                    digitalWrite(STEPPER_MOTOR_PIN, LOW);
                }
            }

            /********(2.5.2) transmit to tx1********/
            // assemble outgoing packet
            // clear the buffer
            for (int i = 0; i < 32; i++) {
                htpz_out_buf[i] = 0x00;
            }
            htpz_out_buf[0] = HTPZ_OUT_HEADER; // set the header

            // set the rc_ctl fields
            for (int i = 10; i < 30; i++) {
                htpz_out_buf[i] = tpz_in_buf[i];
            }

            // transmit to htpz the buffer
            Serial2.write((uint8_t*) htpz_out_buf, 32);
#endif
        }
    }

    // (2.6) process packet from js
    if (Serial3.available() > 0) {
        js_rx_byte = Serial3.read();
        if (js_rx_byte == 0xA5) {
            receive(js_rx_byte);

            // after receive, process data
            float calculated_power = general_info->realChassisOutV * general_info->realChassisOutA;
            js_real_chassis_out_power = (uint16_t) calculated_power;
            js_remain_life_value = general_info->remainLifeValue;
            js_big_rune_0_status = general_info->bigRune0Status;
            js_big_rune_1_status = general_info->bigRune1status;
        }
#if DEBUG
        Serial.println(general_info->realChassisOutV);
#endif
    }

    // (3) transmit to tpz
    unsigned long tpz_timer_current_time = millis();
    if (tpz_timer_current_time - tpz_timer_prev_time >= tpz_timer_period) {
        tpz_timer_prev_time = tpz_timer_current_time;

        /********(3.1) transmit to tpz********/
        // assemble outgoing packet
        // copy incoming buffer to outgoing buffer
        for (int i = 0; i < TX1_TPZ_PACKET_SIZE; i++) {
            tpz_out_buf[i] = tx1_in_buf[i];
        }
        tpz_out_buf[0] = TPZ_OUT_HEADER; // set the header

        // insert kalman data to outgoing buffer
#if MPU_ENABLE
        insert_mpu_kalman_data();
#endif
        // insert js data to outgoing buffer
        tpz_out_buf[26] = L_BYTE(js_real_chassis_out_power);
        tpz_out_buf[27] = H_BYTE(js_real_chassis_out_power);

        // transmit to tpz the buffer
        Serial1.write((uint8_t*) tpz_out_buf, TX1_TPZ_PACKET_SIZE);
    }

    // (4) transmit to tx1
    unsigned long tx1_timer_current_time = millis();
    if (tx1_timer_current_time - tx1_timer_prev_time >= tx1_timer_period) {
        tx1_timer_prev_time = tx1_timer_current_time;

        /********(4.1) transmit to tx1********/
        // assemble outgoing packet
        // clear the buffer
        for (int i = 0; i < 32; i++) {
            tx1_out_buf[i] = 0x00;
        }
        tx1_out_buf[0] = TX1_OUT_HEADER; // set the header

        // insert data
        tx1_out_buf[2] = js_big_rune_0_status;
        tx1_out_buf[3] = js_big_rune_1_status;

        // transmit to tx1 the buffer
#if !DEBUG
        Serial.write((uint8_t*) tx1_out_buf, 32);
#endif
    }

    // (5) misc process
    misc_process();

    delay(1);
}

void misc_process() {
    // set the feeder motor state
    if (feeder_motor_state_req) { // turn on led if feeder motor on
        LED_ON();
        digitalWrite(FEEDER_MOTOR_PIN, HIGH);
    } else {
        LED_OFF();
        digitalWrite(FEEDER_MOTOR_PIN, LOW);
    }
}

void read_one() {
    while (!Serial3.available()){}
    js_rx_byte = Serial3.read();
}

/*
    **description: reciving the data from the judgement system and put them into
    different data structure according to the different data type.

    **input: SOF, in this case the byte A5 received from judgement system

    **output: none, data will be store in the globally declared & initiallized
    data structure
*/
void receive(unsigned char SOF) {

    unsigned char buffer_1[4];
    uint16_t data_size;
    uint16_t data_type;

    buffer_1[0] = SOF;
    int itr = 1;

    while (itr < 4) {
        read_one();
        buffer_1[itr] = js_rx_byte;
        itr++;
    }
    if (Verify_CRC8_Check_Sum(buffer_1, 4)) {
        data_size = buffer_1[1];
        unsigned char buffer_2[SHARED_SIZE + data_size];
        memcpy(buffer_2, buffer_1, 4);
        while (itr < SHARED_SIZE + data_size) {
            read_one();
            buffer_2[itr] = js_rx_byte;
            itr++;
        }

        // int i = 0;
        // while (i < SHARED_SIZE + data_size) {
        //     Serial.print(buffer_2[i], HEX);
        //     Serial.print(" ");
        //     i++;
        // }
        // Serial.println();

        // sometimes it does not pass the CRC test because the data is corrupted
        if (Verify_CRC16_Check_Sum(buffer_2, SHARED_SIZE + data_size)) {
            data_type = buffer_2[4];

            // at this point we have got all the data from the js into the buffer_2
            if (data_type == 1) {
                //Serial.println("data is 1");
                memcpy(general_info, buffer_2 + 6, 20);
                general_info->conveyorBelts0 = buffer_2[26] & 3;
                general_info->conveyorBelts1 = (buffer_2[26] >> 2) & 3;
                general_info->parkingApron0 = (buffer_2[26] >> 4) & 1;
                general_info->parkingApron0 = (buffer_2[26] >> 5) & 1;
                general_info->parkingApron0 = (buffer_2[26] >> 6) & 1;
                general_info->parkingApron0 = (buffer_2[26] >> 7) & 1;
                //memcpy(general_info->gps_data, buffer_2 + 27, 17);
            } else if (data_type == 2) {
                health_data->weakId = buffer_2[6];
                health_data->way = (buffer_2[6] >> 4) & 15;
                memcpy(&(health_data->value), buffer_2 + 7, 2);
            } else if (data_type == 3) {
                memcpy(weapon_data, buffer_2 + 6, data_size);
            }
        }
    }
}

// sets the kalman data into the outgoing buffer
void insert_mpu_kalman_data() {
    // turn the data into integer by multiplying a constant
    kal_int_x = kalAngleX * KAL_CONST_X;
    kal_int_y = kalAngleY * KAL_CONST_Y;
    kal_int_z = kalAngleZ * KAL_CONST_Z;

    tpz_out_buf[20] = L_BYTE(kal_int_x);
    tpz_out_buf[21] = H_BYTE(kal_int_x);
    tpz_out_buf[22] = L_BYTE(kal_int_y);
    tpz_out_buf[23] = H_BYTE(kal_int_y);
    tpz_out_buf[24] = L_BYTE(kal_int_z);
    tpz_out_buf[25] = H_BYTE(kal_int_z);
}

void led_toggle() {
    if (digitalRead(LED_PIN) == LOW) {
        LED_ON();
    } else {
        LED_OFF();
    }
}

void mpu_kalman_init() {
    delay(100); // Wait for sensors to get ready

    //Serial.begin(115200);
    Wire.begin();
    TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(MPU6050, 0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(MPU6050, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(MPU6050, 0x75, i2cData, 1));
    if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
        //Serial.print(F("Error reading sensor"));
        while (1);
    }

    while (i2cWrite(HMC5883L, 0x02, 0x00, true)); // Configure device for continuous mode
    calibrateMag();

    delay(100); // Wait for sensors to stabilize

    /* Set Kalman and gyro starting angle */
    updateMPU6050();
    updateHMC5883L();
    updatePitchRoll();
    updateYaw();

    kalmanX.setAngle(roll); // First set roll starting angle
    gyroXangle = roll;
    compAngleX = roll;

    kalmanY.setAngle(pitch); // Then pitch
    gyroYangle = pitch;
    compAngleY = pitch;

    kalmanZ.setAngle(yaw); // And finally yaw
    gyroZangle = yaw;
    compAngleZ = yaw;

    timer = micros(); // Initialize the timer
}

void mpu_kalman_process() {
    /* Update all the IMU values */
    updateMPU6050();
    updateHMC5883L();

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();


    /* Roll and pitch estimation */
    updatePitchRoll();
    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif


    /* Yaw estimation */
    updateYaw();
    double gyroZrate = gyroZ / 131.0; // Convert to deg/s
    // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
    if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
        kalmanZ.setAngle(yaw);
        compAngleZ = yaw;
        kalAngleZ = yaw;
        gyroZangle = yaw;
    } else
        kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter


    /* Estimate angles using gyro only */
    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter
    //gyroYangle += kalmanY.getRate() * dt;
    //gyroZangle += kalmanZ.getRate() * dt;

    /* Estimate angles using complimentary filter */
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
    compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

    // Reset the gyro angles when they has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;
    if (gyroZangle < -180 || gyroZangle > 180)
        gyroZangle = kalAngleZ;


    /* Print Data */
#if 0
    Serial.print(roll); Serial.print("\t");
    Serial.print(gyroXangle); Serial.print("\t");
    Serial.print(compAngleX); Serial.print("\t");
    Serial.print(kalAngleX); Serial.print("\t");

    Serial.print("\t");

    Serial.print(pitch); Serial.print("\t");
    Serial.print(gyroYangle); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
    Serial.print(kalAngleY); Serial.print("\t");

    Serial.print("\t");

    Serial.print(yaw); Serial.print("\t");
    Serial.print(gyroZangle); Serial.print("\t");
    Serial.print(compAngleZ); Serial.print("\t");
    Serial.print(kalAngleZ); Serial.print("\t");
#endif
#if 0 // Set to 1 to print the IMU data
    Serial.print(accX / 16384.0); Serial.print("\t"); // Converted into g's
    Serial.print(accY / 16384.0); Serial.print("\t");
    Serial.print(accZ / 16384.0); Serial.print("\t");

    Serial.print(gyroXrate); Serial.print("\t"); // Converted into degress per second
    Serial.print(gyroYrate); Serial.print("\t");
    Serial.print(gyroZrate); Serial.print("\t");

    Serial.print(magX); Serial.print("\t"); // After gain and offset compensation
    Serial.print(magY); Serial.print("\t");
    Serial.print(magZ); Serial.print("\t");
#endif
#if 0 // Set to 1 to print the temperature
    Serial.print("\t");

    double temperature = (double)tempRaw / 340.0 + 36.53;
    Serial.print(temperature); Serial.print("\t");
#endif

    //Serial.println();

    //delay(10);
}

void updateMPU6050() {
    while (i2cRead(MPU6050, 0x3B, i2cData, 14)); // Get accelerometer and gyroscope values
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = -((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = -(i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = -(i2cData[12] << 8) | i2cData[13];
}

void updateHMC5883L() {
    while (i2cRead(HMC5883L, 0x03, i2cData, 6)); // Get magnetometer values
    magX = ((i2cData[0] << 8) | i2cData[1]);
    magZ = ((i2cData[2] << 8) | i2cData[3]);
    magY = ((i2cData[4] << 8) | i2cData[5]);
}

void updatePitchRoll() {
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    roll = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
    magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
    magZ *= -1;

    magX *= magGain[0];
    magY *= magGain[1];
    magZ *= magGain[2];

    magX -= magOffset[0];
    magY -= magOffset[1];
    magZ -= magOffset[2];

    double rollAngle = kalAngleX * DEG_TO_RAD;
    double pitchAngle = kalAngleY * DEG_TO_RAD;

    double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
    double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
    yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

    yaw *= -1;
}

void calibrateMag() { // Inspired by: https://code.google.com/p/open-headtracker/
    i2cWrite(HMC5883L, 0x00, 0x11, true);
    delay(100); // Wait for sensor to get ready
    updateHMC5883L(); // Read positive bias values

    int16_t magPosOff[3] = { magX, magY, magZ };

    i2cWrite(HMC5883L, 0x00, 0x12, true);
    delay(100); // Wait for sensor to get ready
    updateHMC5883L(); // Read negative bias values

    int16_t magNegOff[3] = { magX, magY, magZ };

    i2cWrite(HMC5883L, 0x00, 0x10, true); // Back to normal

    magGain[0] = -2500 / float(magNegOff[0] - magPosOff[0]);
    magGain[1] = -2500 / float(magNegOff[1] - magPosOff[1]);
    magGain[2] = -2500 / float(magNegOff[2] - magPosOff[2]);

#if 0
    Serial.print("Mag cal: ");
    Serial.print(magNegOff[0] - magPosOff[0]);
    Serial.print(",");
    Serial.print(magNegOff[1] - magPosOff[1]);
    Serial.print(",");
    Serial.println(magNegOff[2] - magPosOff[2]);

    Serial.print("Gain: ");
    Serial.print(magGain[0]);
    Serial.print(",");
    Serial.print(magGain[1]);
    Serial.print(",");
    Serial.println(magGain[2]);
#endif
}
