#ifndef BMP180_H__
#define BMP180_H__

#include <stdint.h>

#define BMP180_ADDR (0x77)
#define TWI_INSTANCE_ID     0

typedef struct {
    uint16_t AC1;
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

}BMP180_CalibrationData_t;

typedef struct{
    uint8_t chip_id;
    uint8_t sampling_mode;
    float altitude;
    float pressure;
    float temperature;
    float baseline;
    float elevation;
    int oversample; 
    BMP180_CalibrationData_t calib_data;
}BMP180_t;

void BMP180_Init(BMP180_t * const bmp, float elevation);
void BMP180_ReadChipId(BMP180_t * const bmp);
int BMP180_CheckChipId(BMP180_t * const bmp);
void BMP180_ReadTemperature(BMP180_t * const bmp);


#endif // BMP180_H__