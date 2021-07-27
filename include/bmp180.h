/**
 * @brief
    bmp180 is a c module for the Bosch bmp180 sensor. It measures
    temperature as well as pressure, with a high enough resolution to calculate
    altitude.
    data-sheet: BST-BMP180-DS000-09.pdf
 */

#ifndef BMP180_H__
#define BMP180_H__

#include <stdint.h>

#define BMP180_ADDR (0x77)
#define TWI_INSTANCE_ID     0

typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;

    long B5;

}BMP180_CalibrationData_t;

typedef struct{
    uint8_t chip_id;
    uint8_t sampling_mode;
    double altitude;
    float pressure;
    float temperature;
    float sea_level; /* https://keisan.casio.com/exec/system/1224575267 */
    float elevation;
    int oversample; 
    BMP180_CalibrationData_t calib_data;
}BMP180_t;

void BMP180_Init(BMP180_t * const bmp, float elevation, float sea_level);
void BMP180_ReadChipId(BMP180_t * const bmp);
int BMP180_CheckChipId(BMP180_t * const bmp);
void BMP180_ReadTemperature(BMP180_t * const bmp);
void BMP180_ReadPressure(BMP180_t * const bmp);
void BMP180_ReadAltitude(BMP180_t * const bmp);
void BMP180_Reset(BMP180_t * const bmp);


#endif // BMP180_H__