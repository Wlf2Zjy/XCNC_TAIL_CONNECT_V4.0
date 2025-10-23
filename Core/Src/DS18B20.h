#ifndef __DS18B20_H
#define __DS18B20_H

#include "main.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>

/* -------------------- ������ö�ٶ��� -------------------- */
typedef enum {
    DS18B20_MAIN = 0,  // ���¶ȴ����� - PC13
    DS18B20_LEFT,       // ���¶ȴ����� - PC14
    DS18B20_RIGHT       // ���¶ȴ����� - PC15
} DS18B20_Sensor_t;

/* -------------------- �������� -------------------- */
void DS18B20_Init(DS18B20_Sensor_t sensor);
uint8_t DS18B20_Start(DS18B20_Sensor_t sensor);
void DS18B20_WriteByte(DS18B20_Sensor_t sensor, uint8_t dat);
uint8_t DS18B20_ReadByte(DS18B20_Sensor_t sensor);
float DS18B20_ReadTemp(DS18B20_Sensor_t sensor);
void DS18B20_ReadAllTemps(float *main_temp, float *left_temp, float *right_temp);
const char* DS18B20_GetSensorName(DS18B20_Sensor_t sensor);

#endif
