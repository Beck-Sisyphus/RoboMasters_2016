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


/* 
	CRC8 check code, provided by DJI.
	crc8 generator polynomial:G(x)=x8+x5+x4+1
*/

const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8) {

	unsigned char ucIndex;

	while (dwLength--) {
		ucIndex = ucCRC8^(*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}

	return(ucCRC8);
}


/*
	** Descriptions: CRC8 Verify function
	** Input: Data to Verify,Stream length = Data + checksum
 	** Output: True or False (CRC Verify Result)
*/

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) {

	unsigned char ucExpected = 0;

	if ((pchMessage == 0) || (dwLength <= 2)) 
		return 0;
	
	ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
	return ( ucExpected == pchMessage[dwLength-1] );
}


/*
	CRC8 check code ends, CRC16 check code begins
*/

uint16_t CRC_INIT = 0xffff;

const uint16_t wCRC_Table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/*
	** Descriptions: CRC16 checksum function
 	** Input: Data to check,Stream length, initialized checksum
 	** Output: CRC checksum
*/

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {

	uint8_t chData;

	if (pchMessage == NULL) {
		return 0xFFFF;
	}

	while(dwLength--) {
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
	}

	return wCRC;
}


/*
	** Descriptions: CRC16 Verify function
 	** Input: Data to Verify,Stream length = Data + checksum
	** Output: True or False (CRC Verify Result)
*/

uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) {

	uint16_t wExpected = 0;

	if ((pchMessage == NULL) || (dwLength <= 2)) {
		return 0;
	}

	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && 
			((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
	CRC16 check code ends.
*/


// data structurs which store the information obtained from judgement system
GENERAL_INFO *general_info;
HEALTH_DATA *health_data;
WEAPON_DATA *weapon_data;


// The byte read from judgement system

unsigned char rx_byte;


// read one byte from judgement system

void read_one();


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
    	buffer_1[itr] = rx_byte;
    	itr++;
    }
    if (Verify_CRC8_Check_Sum(buffer_1, 4)) {
    	data_size = buffer_1[1];
    	unsigned char buffer_2[SHARED_SIZE + data_size];
    	memcpy(buffer_2, buffer_1, 4);
    	while (itr < SHARED_SIZE + data_size) {
        	read_one();
        	buffer_2[itr] = rx_byte;
        	itr++;
      	}
	
		// sometimes it does not pass the CRC test because the data is corrupted
      	if (Verify_CRC16_Check_Sum(buffer_2, SHARED_SIZE + data_size)) {        	
			// data type is packed as 0100 not 0001
			data_type = buffer_2[4];
		
			// at this point we have got all the data from the js into the buffer_2 
        	if (data_type == 1) {
          		memcpy(general_info, buffer_2 + 6, 20);
          		general_info->conveyorBelts0 = buffer_2[26] & 3;
          		general_info->conveyorBelts1 = (buffer_2[26] >> 2) & 3;
          		general_info->parkingApron0 = (buffer_2[26] >> 4) & 1;
          		general_info->parkingApron0 = (buffer_2[26] >> 5) & 1;
          		general_info->parkingApron0 = (buffer_2[26] >> 6) & 1;
          		general_info->parkingApron0 = (buffer_2[26] >> 7) & 1;
          		
          		// works on my board
          		memcpy(general_info->gps_data, buffer_2 + 27, 17);
        	} else if (data_type == 2) {
          		health_data->weakId = buffer_2[6];  
          		health_data->way = (buffer_2[6] >> 4) & 15;
          		memcpy(&(health_data->value), buffer_2 + 7, 2);
        	} else if (data_type == 3) {
          		memcpy(weapon_data, buffer_2 + 6, data_size);
        	}
        	
        	// send the whatever kind of data package it received through Serial3/(can't use parallel)
        	int i = 6;
        	while (i < 6 + data_size) {
        		Serial3.write(buffer_2[i]);
        		i++;
        	}
      	}
    }
}

void read_one() {
  while (!Serial3.available()){}
  rx_byte = Serial3.read();
}

void setup() {

  Serial.begin(115200);
  //while (!Serial.available()){}
  Serial3.begin(115200);

  general_info = (GENERAL_INFO *) malloc(sizeof(general_info));

  general_info->gps_data = (GPS_DATA *) malloc(sizeof(GPS_DATA));

  health_data = (HEALTH_DATA *) malloc(sizeof(health_data));

  weapon_data = (WEAPON_DATA *) malloc(sizeof(weapon_data));

}

/*
	keep reading from the judgment system until it receive the SOF, then it will
	process the data from the judgment system and store it into the data structure.
*/
void loop() {
  while (1) {
    read_one();
    if (rx_byte == 165) {
      receive(rx_byte);
    }
  }
}
