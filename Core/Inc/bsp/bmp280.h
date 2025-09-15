#ifndef BMP280_H_
#define BMP280_H_

#include "stm32f4xx_hal.h"   // или твой чип
#include <stdint.h>

#define BMP280_ADDR  (0x76 << 1)  // I2C-адрес

// Структура для хранения калибровочных коэффициентов
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t  t_fine;
    I2C_HandleTypeDef *hi2c; // указатель на I2C-хэндл
} BMP280_t;

// ==== API ====

// Инициализация
void BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c);

// Чтение калибровочных данных
void BMP280_ReadCalibrationData(BMP280_t *dev);

// Снятие показаний
float BMP280_ReadTemperature(BMP280_t *dev);
float BMP280_ReadPressure(BMP280_t *dev);

#endif /* BMP280_H_ */
