#ifndef __DJI_JS_H__
#define __DJI_JS_H__

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define SHARED_SIZE 8
#define PTR_SIZE 1
#define PTR_TYPE 4

/* 
	holds the information about the robot's position
*/

typedef struct __attribute__((packed)) {
	uint8_t flag;
	uint32_t x;
	uint32_t y;
	uint32_t z;
	uint32_t compass;
} GPS_DATA;


/*
	holds the general information of the robot's status including
	remaining time, remaining life, chassis voltage & current, rune status,
	conveyorBelts & airport status and robot's location information.	
*/

typedef struct __attribute__((packed)) {
	uint32_t remainTime;
	uint16_t remainLifeValue;
	float realChassisOutV;
	float realChassisOutA;
	uint8_t runeStatus[4];
	uint8_t bigRune0Status;
	uint8_t bigRune1status;
	uint8_t conveyorBelts0:2;
	uint8_t conveyorBelts1:2;
	uint8_t parkingApron0:1;
	uint8_t parkingApron1:1;
	uint8_t parkingApron2:1;
	uint8_t parkingApron3:1;
	GPS_DATA *gps_data;
} GENERAL_INFO;


/*
	holds the information about health change, it could tell which armor
	is hitten, the cause of the health decrease and the value of health decrease.
*/

typedef struct __attribute__((packed)) {
	uint8_t weakId:4;
	uint8_t way:4;
	uint16_t value;
} HEALTH_DATA;


/*
	holds the information about the weapon, includes the projectile's speed and
	frequency for both 17mm bullet and golf.
*/

typedef struct __attribute__((packed)) {
	float realBulletShootSpeed;
	float realBulletShootFreq;
	float realGolfShootSpeed;
	float realGolfShootFreq;
} WEAPON_DATA;


unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

#endif
