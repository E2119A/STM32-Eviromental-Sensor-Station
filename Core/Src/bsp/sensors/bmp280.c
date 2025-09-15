#include "bmp280.h"

// ====== Внутренние функции ======
static uint8_t bmp280_read8(BMP280_t *dev, uint8_t reg)
{
    uint8_t val;
    HAL_I2C_Mem_Read(dev->hi2c, BMP280_ADDR, reg, 1, &val, 1, HAL_MAX_DELAY);
    return val;
}

static uint16_t bmp280_read16(BMP280_t *dev, uint8_t reg)
{
    uint8_t buf[2];
    HAL_I2C_Mem_Read(dev->hi2c, BMP280_ADDR, reg, 1, buf, 2, HAL_MAX_DELAY);
    return (buf[1] << 8) | buf[0];
}

static int16_t bmp280_readS16(BMP280_t *dev, uint8_t reg)
{
    return (int16_t)bmp280_read16(dev, reg);
}

// ====== API ======
void BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c)
{
    dev->hi2c = hi2c;

    // Нормальный режим, oversampling x1
    uint8_t ctrl_meas = 0x27;
    uint8_t config = 0x00;
    HAL_I2C_Mem_Write(dev->hi2c, BMP280_ADDR, 0xF4, 1, &ctrl_meas, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(dev->hi2c, BMP280_ADDR, 0xF5, 1, &config, 1, HAL_MAX_DELAY);

    BMP280_ReadCalibrationData(dev);
}

void BMP280_ReadCalibrationData(BMP280_t *dev)
{
    dev->dig_T1 = bmp280_read16(dev, 0x88);
    dev->dig_T2 = bmp280_readS16(dev, 0x8A);
    dev->dig_T3 = bmp280_readS16(dev, 0x8C);
    dev->dig_P1 = bmp280_read16(dev, 0x8E);
    dev->dig_P2 = bmp280_readS16(dev, 0x90);
    dev->dig_P3 = bmp280_readS16(dev, 0x92);
    dev->dig_P4 = bmp280_readS16(dev, 0x94);
    dev->dig_P5 = bmp280_readS16(dev, 0x96);
    dev->dig_P6 = bmp280_readS16(dev, 0x98);
    dev->dig_P7 = bmp280_readS16(dev, 0x9A);
    dev->dig_P8 = bmp280_readS16(dev, 0x9C);
    dev->dig_P9 = bmp280_readS16(dev, 0x9E);
}

float BMP280_ReadTemperature(BMP280_t *dev)
{
    int32_t adc_T = ((uint32_t)bmp280_read8(dev, 0xFA) << 12) |
                    ((uint32_t)bmp280_read8(dev, 0xFB) << 4) |
                    ((bmp280_read8(dev, 0xFC) >> 4) & 0x0F);

    int32_t var1, var2;
    var1 = ((((adc_T>>3) - ((int32_t)dev->dig_T1<<1))) * ((int32_t)dev->dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dev->dig_T1)) * ((adc_T>>4) - ((int32_t)dev->dig_T1))) >> 12) * ((int32_t)dev->dig_T3)) >> 14;

    dev->t_fine = var1 + var2;
    float T  = (dev->t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

float BMP280_ReadPressure(BMP280_t *dev)
{
    int32_t adc_P = ((uint32_t)bmp280_read8(dev, 0xF7) << 12) |
                    ((uint32_t)bmp280_read8(dev, 0xF8) << 4) |
                    ((bmp280_read8(dev, 0xF9) >> 4) & 0x0F);

    int64_t var1, var2, p;
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1*(int64_t)dev->dig_P5)<<17);
    var2 = var2 + (((int64_t)dev->dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3)>>8) + ((var1 * (int64_t)dev->dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dev->dig_P1)>>33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125)/var1;
    var1 = (((int64_t)dev->dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dev->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7)<<4);
    return (float)p / 25600.0f;
}
