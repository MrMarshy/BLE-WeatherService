#include "bmp180.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "boards.h"
#include <string.h>

#if 0
static const char* CALIB_STR[] = {
    "AC1", "AC2", "AC3", "AC4", "AC5", "AC6",
    "B1", "B2",
    "MB", "MC", "MD"
};
#endif


/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* Indicates if twi has been enabled */
static bool is_twi_enabled = false;


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(const uint8_t type){
}

static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

static bool BMP180_RegisterRead(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes);

static bool BMP180_RegisterWrite(uint8_t register_address, uint8_t value);

static void BMP180_ReadCalibrationData(BMP180_t * const bmp);

static void enable_twi(void){

    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_bmp180_config = {
        .scl                = SCL_PIN,
        .sda                = SDA_PIN,
        .frequency          = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_MID,
        .clear_bus_init     = true
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_bmp180_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

    is_twi_enabled = true;

}

static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

void BMP180_Init(BMP180_t * const bmp, float elevation){
    
    if(!bmp){
        NRF_LOG_ERROR("Unable to initalise BMP180 sensor.");
        while(1);
    }

    enable_twi();

    if(!is_twi_enabled){
        NRF_LOG_ERROR("Unable to initalise NRF52 TWI");
        NRF_BREAKPOINT;
    }
    
    bmp->chip_id = 0;
    bmp->oversample = 3;
    bmp->baseline = 1013.25f;
    bmp->altitude = 0.0f;
    bmp->pressure = 0.0f;
    bmp->temperature = 0.0f;
    bmp->elevation = elevation;
    
    memset(&bmp->calib_data, 0, sizeof(bmp->calib_data));

    BMP180_ReadChipId(bmp);

    if(BMP180_CheckChipId(bmp) != 0){
        while(1);
    }
    BMP180_ReadCalibrationData(bmp);
}

/**
 * @brief Function for reading the chip id of the BMP180 sensor.
 *
 * @param[in] bmp          Pointer to BMP180_t structure.
 */
void BMP180_ReadChipId(BMP180_t * const bmp){

    if(!bmp){
        NRF_LOG_ERROR("BMP180 Sensor is not intialised.");
        return;
    }

    if(!is_twi_enabled){
        NRF_LOG_ERROR("NRF52 TWI is not initialised.");
        return;
    }

    uint8_t ID_REG = 0xD0;

    if(!BMP180_RegisterRead(ID_REG, &bmp->chip_id, 1)){
        NRF_LOG_INFO("Unable to read BMP180 register, check wiring?");
    }

}

int BMP180_CheckChipId(BMP180_t * const bmp){
    
    if(!bmp){
        NRF_LOG_ERROR("BMP180_CheckChipId function must be passed valid pointer");
        return -1;
    }

    if(bmp->chip_id == 0x55){
        NRF_LOG_INFO("Successfully read correct BMP180 chip id");
        NRF_LOG_INFO("BMP180 Chip id is 0x%x", bmp->chip_id);
        return 0;
    }
    else{
        NRF_LOG_ERROR("Unable to read correct BMP180 chip id - check wiring");
        return -1;
    }
}

/**
 * Function to read the calibration data stored on the BMP180 EEPROM 
 */
static void BMP180_ReadCalibrationData(BMP180_t * const bmp){
    if(!bmp){
        return;
    }

    if(!is_twi_enabled){
        return;
    }

    uint8_t const CALIBRATION_REGS []={ 
        0xAA, 0xAB, /* AC1 */
        0xAC, 0xAD, /* AC2 */
        0xAE, 0xAF, /* AC3 */
        0xB0, 0xB1, /* AC4 */
        0xB2, 0xB3, /* AC5 */
        0xB4, 0xB5, /* AC6 */
        0xB6, 0xB7, /* B1 */
        0xB8, 0xB9, /* B2 */
        0xBA, 0xBB, /* MB */
        0xBC, 0xBD, /* MC */
        0xBE, 0xBF  /* MD */
    };

    uint8_t result [22]= {0};
    
    for(int i = 0; i < 22; i+=2){
        
        BMP180_RegisterRead(CALIBRATION_REGS[i], &result[i], 1);
        
        BMP180_RegisterRead(CALIBRATION_REGS[i + 1], &result[i + 1], 1);

        switch (i){
            case 0: // AC1
                bmp->calib_data.AC1 = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1];
                break;
            case 2: // AC2
                bmp->calib_data.AC2 = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1]; 
                break;
            case 4: // AC3
                bmp->calib_data.AC3 = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1];
                break;
            case 6: // AC4
                bmp->calib_data.AC4 = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1];
                break;
            case 8: // AC5
                bmp->calib_data.AC5 = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1];
                break;
            case 10: // AC6
                bmp->calib_data.AC6 = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1];
                break;
            case 12: // B1
                bmp->calib_data.B1 = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1]; 
                break;
            case 14: // B2
                bmp->calib_data.B2 = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1]; 
                break;
            case 16: // MB
                bmp->calib_data.MB = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1]; 
                break;
            case 18: // MC
                bmp->calib_data.MC = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1]; 
                break;
            case 20: // MD
                bmp->calib_data.MD = ((uint8_t)(result[i] << 8U)) | (uint8_t)result[i + 1]; 
                break;
            default:
                ;
        }
#if 0
        NRF_LOG_INFO("CALIB VALUE %s: %d", CALIB_STR[0], bmp->calib_data.AC1);
        NRF_LOG_INFO("CALIB VALUE %s: %d", CALIB_STR[1], bmp->calib_data.AC2);
        NRF_LOG_INFO("CALIB VALUE %s: %d", CALIB_STR[2], bmp->calib_data.AC3);
        NRF_LOG_INFO("CALIB VALUE %s: %u", CALIB_STR[3], bmp->calib_data.AC4);
        NRF_LOG_INFO("CALIB VALUE %s: %u", CALIB_STR[4], bmp->calib_data.AC5);
        NRF_LOG_INFO("CALIB VALUE %s: %u", CALIB_STR[5], bmp->calib_data.AC6);
        NRF_LOG_INFO("CALIB VALUE %s: %d", CALIB_STR[6], bmp->calib_data.B1);
        NRF_LOG_INFO("CALIB VALUE %s: %d", CALIB_STR[7], bmp->calib_data.B2);
        NRF_LOG_INFO("CALIB VALUE %s: %d", CALIB_STR[8], bmp->calib_data.MB);
        NRF_LOG_INFO("CALIB VALUE %s: %d", CALIB_STR[9], bmp->calib_data.MC);
        NRF_LOG_INFO("CALIB VALUE %s: %d", CALIB_STR[10], bmp->calib_data.MD);
#endif
    }

}

void BMP180_ReadTemperature(BMP180_t * const bmp){
    
    if(!bmp){
        return;
    }

    /* Read Uncompensated temperature value */
    uint8_t const value = 0x2E;
    uint8_t const CNRTL_MEAS_REG = 0xF4;
    BMP180_RegisterWrite(CNRTL_MEAS_REG, value);
    nrf_delay_us(5000); // Wait >= 4.5ms as per datasheet.

    uint8_t result[2] = {0};
    uint8_t const OUT_MSB = 0xF6;
    uint8_t const OUT_LSB = 0xF7;
    BMP180_RegisterRead(OUT_MSB, &result[0], 1); // MSB
    BMP180_RegisterRead(OUT_LSB, &result[1], 1); // LSB
    long UT = ((long)result[0] << 8) + (long)result[1]; // As per datasheet

    /* Calculate True Temperature */
    long X1 = ((UT - (long)bmp->calib_data.AC6) * (long)bmp->calib_data.AC5 ) >> 15;
    long X2 = ((long)bmp->calib_data.MC << 11) / (X1 + (long)bmp->calib_data.MD);
    long B5 = X1 + X2;
    bmp->temperature = (float)((B5 + 8L) >> 4) / 10.0f;

    NRF_LOG_INFO("Temperature is %d deg Celsius", (int32_t)bmp->temperature);
    
}

/*
  A Function to read data from the BMP180 Sensor
*/ 
static bool BMP180_RegisterRead(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes){
    ret_code_t err_code;

    //Set the flag to false to show the receiving is not yet completed
    m_xfer_done = false;
    
    // Send the Register address where we want to write the data
    err_code = nrf_drv_twi_tx(&m_twi, BMP180_ADDR, &register_address, number_of_bytes, true);
	  
    //Wait for the transmission to get completed
    while (m_xfer_done == false){}
    
    // If transmission was not successful, exit the function with false as return value
    if (NRF_SUCCESS != err_code){
        return false;
    }

    //set the flag again so that we can read data from the MPU6050's internal register
    m_xfer_done = false;
	  
    // Receive the data from the BMP180
    err_code = nrf_drv_twi_rx(&m_twi, BMP180_ADDR, destination, number_of_bytes);
		
    //wait until the transmission is completed
    while (m_xfer_done == false){}
	
    // if data was successfully read, return true else return false
    if (NRF_SUCCESS != err_code){
        return false;
    }
    
    return true;
}

/*
   A function to write a Single Byte to BMP180 internal Registers
*/ 
static bool BMP180_RegisterWrite(uint8_t register_address, uint8_t value){
    ret_code_t err_code;
    uint8_t tx_buf[2];
	
    //Write the register address and data into transmit buffer
    tx_buf[0] = register_address;
    tx_buf[1] = value;

    //Set the flag to false to show the transmission is not yet completed
    m_xfer_done = false;
    
    //Transmit the data over TWI Bus
    err_code = nrf_drv_twi_tx(&m_twi, BMP180_ADDR, tx_buf, 2, false);
    
    //Wait until the transmission of the data is finished
    while (m_xfer_done == false);

    // if there is no error then return true else return false
    if (NRF_SUCCESS != err_code){
        return false;
    }
    
    return true;	
}


/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}