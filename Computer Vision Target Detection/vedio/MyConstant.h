#ifndef MYCONSTANT_H
#define MYCONSTANT_H

#ifdef ERROR
#define ERROR 0
#endif
#ifdef TRUE
#define TRUE 1
#endif
//函数输出模式
#define PRINT_PARAM 1
#define NOT_PRINT 0

//捕获模式
#define FROM_FILE 0
#define FROM_CAMERA 1

//逻辑状态定义
#define LEFT 0
#define RIGHT 1
#define NOT_ENTIRE 0
#define ENTIRE 1
#define NOT_APPEAR 0
#define APPEAR 1

//内参数定义
#define Fx 871.21871
#define Fy 871.09577	
#define Cx 331.52937
#define Cy 229.47270

#define K1 0.00604
#define K2 -0.22564
#define P1 0.00035
#define P2 0.00387


//随测试修改常量
#define TIME_TO_SAVE 7
#define X_START 0
#define X_END 640
#define Y_START 105
#define Y_END 225
#define TRANSLENGTH 20
#define TRANS 4


#endif