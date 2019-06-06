#pragma once

#include "arm_math.h"
#include <stdint.h>
#include "generic_i2c_types.h"

#pragma once



#pragma once

//#define MAXIM_DEBUG

#define MAXIM_I2C_ADDRESS         ( 0x57 )
#define MAXIM_I2C_BAUDRATE        ( I2C_BAUDRATE )
#define MAXIM_BYTES_PER_ADC_VALUE ( 3 )
#define MAXIM_FIFO_DEPTH          ( 128 )


#define MAXIM_NEW_SAMPLE_CHUNK    ( 256 )
#define MAXIM_INIT_SAMPLE_SIZE 	  ( MAXIM_NEW_SAMPLE_CHUNK )    // 256
#define MAXIM_FINAL_SAMPLE_SIZE   ( MAXIM_NEW_SAMPLE_CHUNK*8 )  // 2048

#define MAXIM_HR_FREQ_MIN ( 0.75 )
#define MAXIM_HR_FREQ_MAX ( 3 )
#define MAXIM_PEAK_THRESHOLD ( 1000 )
#define MAXIM_FIFO_OVERFLOW  ( -1 )

// FIFO bit-mask
#define MAXIM_FIFO_BITMASK ( 0x0003FFFF )

// FIFO bit-shifts, depending on the ADC resolution
#define MAXIM_FIFO_BITSHIFT_18 ( 0 )
#define MAXIM_FIFO_BITSHIFT_17 ( 1 )
#define MAXIM_FIFO_BITSHIFT_16 ( 2 )
#define MAXIM_FIFO_BITSHIFT_15 ( 3 )

/** registers info */

#define MAXIM_REG_INT_STATUS_1 ( 0x00 )
#define MAXIM_REG_INT_STATUS_2 ( 0x01 )

#define MAXIM_REG_INTERRUPT_ENABLE_1 ( 0x02 )
#define MAXIM_REG_INTERRUPT_ENABLE_2 ( 0x03 )

#define MAXIM_REG_FIFO_WR_PTR ( 0x04 )
#define MAXIM_REG_FIFO_OV_PTR ( 0x05 )
#define MAXIM_REG_FIFO_RD_PTR ( 0x06 )
#define MAXIM_REG_FIFO_DATA   ( 0x07 )

#define MAXIM_REG_FIFO_CFG ( 0x8 )
#define MAXIM_REG_MODE_CFG ( 0x9 )
#define MAXIM_REG_SPO2_CFG ( 0xA )

#define MAXIM_REG_LED_RED_PA   ( 0xC )
#define MAXIM_REG_LED_IR_PA    ( 0xD )
#define MAXIM_REG_LED_GREEN_PA ( 0xE )
#define MAXIM_REG_PROXY_PA     ( 0x10 )

#define MAXIM_PROXY_THR        ( 1000 )

#define MAXIM_REG_MULTILED_MODE_CR_12 ( 0x11 )
#define MAXIM_REG_MULTILED_MODE_CR_34 ( 0x12 )

#define MAXIM_REG_TEMP_INT    ( 0x1F )
#define MAXIM_REG_TEMP_FRAC   ( 0x20 )
#define MAXIM_REG_TEMP_CONFIG ( 0x21 )

#define MAXIM_REG_PROXY_INT_THR ( 0x30 )

#define MAXIM_REG_ID_REV  ( 0xFE )
#define MAXIM_REG_ID_PART ( 0xFF )

#define MAXIM_SLOT_NUM ( 4 )
/** bit fields info */

/** interrupt enable */

#define MAXIM_EN_IRQ_BIT_FIFO_ALMOST_FULL ( 1 << 7 )
#define MAXIM_EN_IRQ_BIT_NEW_SAMPLE_RDY   ( 1 << 6 )
#define MAXIM_EN_IRQ_BIT_AMBIENT_OVF      ( 1 << 5 )
#define MAXIM_EN_IRQ_BIT_PROX_INT         ( 1 << 4 )
#define MAXIM_EN_IRQ_BIT_INT_TEMP_RDY     ( 1 << 1 )

/** interrupt status */

#define MAXIM_DATA_RDY_BIT_FIFO_ALMOST_FULL ( 1 << 7 )
#define MAXIM_DATA_RDY_BIT_NEW_SAMPLE_RDY   ( 1 << 6 )
#define MAXIM_DATA_RDY_BIT_AMBIENT_OVF      ( 1 << 5 )
#define MAXIM_DATA_RDY_BIT_PROX_INT         ( 1 << 4 )
#define MAXIM_DATA_RDY_BIT_INT_TEMP_RDY     ( 1 << 1 )

/** FIFO pointers */

#define MAXIM_FIFO_WR_PTR_SHIFT ( 0 )
#define MAXIM_FIFO_WR_PTR_MASK  ( 0x1F << MAXIM_FIFO_WR_PTR_SHIFT )

#define MAXIM_FIFO_RD_PTR_SHIFT ( 0 )
#define MAXIM_FIFO_RD_PTR_MASK  ( 0x1F << MAXIM_FIFO_RD_PTR_SHIFT )

#define MAXIM_FIFO_OVF_CNT_PTR_SHIFT ( 0 )
#define MAXIM_FIFO_OVF_CNT_PTR_MASK  ( 0x1F << MAXIM_FIFO_OVF_CNT_PTR_SHIFT )

/** FIFO configuration */

#define MAXIM_FIFO_CFG_A_FULL_SHIFT ( 0 )
#define MAXIM_FIFO_CFG_A_FULL_MASK  ( 0xF << MAXIM_FIFO_CFG_A_FULL_SHIFT )

#define MAXIM_FIFO_CFGBIT_FIFO_ROLLOVER ( 1 << 4 )

#define MAXIM_FIFO_CFG_FIFO_OVS_SHIFT ( 5 )
#define MAXIM_FIFO_CFG_FIFO_OVS_MASK  ( 0x7 << MAXIM_FIFO_CFG_FIFO_OVS_SHIFT )

/** mode configuration */

#define MAXIM_MODE_CFGBIT_SLEEP ( 1 << 7 )
#define MAXIM_MODE_CFGBIT_RST   ( 1 << 6 )

#define MAXIM_MODE_SHIFT ( 0 )
#define MAXIM_MODE_MASK  ( 0x7 << MAXIM_MODE_SHIFT )

// sample rate in sample/sec
#define MAXIM_CFG_SR_SHIFT ( 2 )
#define MAXIM_CFG_SR_MASK  ( 0x7 << MAXIM_CFG_SR_SHIFT )

// ADC range
#define MAXIM_CFG_ADC_SHIFT ( 5 )
#define MAXIM_CFG_ADC_MASK  ( 0x3 << MAXIM_CFG_ADC_SHIFT )

/** LED pulse amplitude */

// pulse width is in [us] and also determines the ADC resolution
#define MAXIM_CFG_LED_PW_SHIFT ( 0 )
#define MAXIM_CFG_LED_PW_MASK  ( 0x3 << MAXIM_CFG_LED_PW_SHIFT )

/** Multi-LED mode control registers */

// slot modes
#define MAXIM_SLOT_1_3_SHIFT ( 0 )
#define MAXIM_SLOT_1_3_MASK  ( 0x7 << MAXIM_SLOT_1_3_SHIFT )
#define MAXIM_SLOT_2_4_SHIFT ( 4 )
#define MAXIM_SLOT_2_4_MASK  ( 0x7 << MAXIM_SLOT_2_4_SHIFT )

/** temperature data */
#define MAXIM_TEMP_EN ( 1 )


typedef struct
{
  /**
   * I2C relevant information
   */
  genericI2cHandle_t protocol;

} handleMAXIM_t;

typedef enum
{
  modeHR        = 0x2,
  modeSPO2      = 0x3,
  modeMultiLED  = 0x7

} modeMAXIM_t;

typedef enum
{
  ledPower_69  = 0x0, // ADC -> 15 bit
  ledPower_118 = 0x1, // ADC -> 16 bit
  ledPower_215 = 0x2, // ADC -> 17 bit
  ledPower_411 = 0x3  // ADC -> 18 bit

} pwMAXIM_t;

typedef enum
{
  sampleRate_50   = 0x0,
  sampleRate_100  = 0x1,
  sampleRate_200  = 0x2,
  sampleRate_400  = 0x3,
  sampleRate_800  = 0x4,
  sampleRate_1000 = 0x5,
  sampleRate_1600 = 0x6,
  sampleRate_3200 = 0x7

} sampleRateMAXIM_t;

typedef enum
{
  LEDSlotNone  =  0x0,
  LEDSlotRed   =  0x1,
  LEDSlotIR    =  0x2,
  LEDSlotGreen =  0x3

} LEDSlotMAXIM_t;

typedef enum
{
  STATUS_MAXIM_SUCCESS,        // success
  STATUS_MAXIM_ERROR,          // fail
  STATUS_MAXIM_PROTOCOL_ERROR, // protocol failure
  STATUS_MAXIM_INIT_ERROR,     // initialization error
  STATUS_MAXIM_PARAM_ERROR,    // invalid parameter is given
  STATUS_MAXIM_OVF_ERROR,      // sensor data overflow
  STATUS_MAXIM_TIMEOUT         // timeout occured

} maxim_status_t;

/**
 * use evaluation board software and datasheet to determine the desired current value
 */
typedef uint8_t LEDCurrentMAXIM_t;
typedef uint32_t dataMAXIM_t;


typedef enum
{
  adcRange2048  = 0x0,
  adcRange4096  = 0x1,
  adcRange8192  = 0x2,
  adcRange16384 = 0x3

} adcRangeMAXIM_t;

typedef enum
{
  ovsNone = 0x0, // no averaging
  ovs2    = 0x1, // average 2 samples
  ovs4    = 0x2, // average 4 samples
  ovs8    = 0x3, // average 8 samples
  ovs16   = 0x4, // average 16 samples
  ovs32   = 0x5  // average 32 samples

} ovsMAXIM_t;
/**
 * general module settings
 */

typedef struct
{
  // mode (HR, SpO2, or multi-led)
  modeMAXIM_t mode;

  // multiLED slots
  LEDSlotMAXIM_t LEDSlot[ MAXIM_SLOT_NUM ];
         uint8_t activeSlots;

  // LED currents
  LEDCurrentMAXIM_t LEDCurrentRed;
  LEDCurrentMAXIM_t LEDCurrentIR;
  LEDCurrentMAXIM_t LEDCurrentGreen;

  // timings
  sampleRateMAXIM_t sampleRate;
          pwMAXIM_t pulseWidth;

  // ADC range
    adcRangeMAXIM_t range;

  // averaging
        ovsMAXIM_t oversample;

  // device I2C address
  uint8_t address;
 uint16_t baudRate_kbps; // I2C baud-rate

} settingsMAXIM_t;

typedef void (*maxim_callback_t)( void* sampleBuf, uint8_t numAvailSam, void* param );

maxim_status_t MAXIM_Init(
                                  handleMAXIM_t* maximHandle,
                          const settingsMAXIM_t* maximSettings
                        );

maxim_status_t MAXIM_Deinit();


maxim_status_t MAXIM_Reset();

maxim_status_t MAXIM_ReadRawData(
                                    uint8_t* dataBuff,
                                    uint8_t* sampleNum
                                );

void hr_fmtDef2Float(
                      const void* src,
                            void* dst,
                         uint32_t idx
                    );

void hr_fmtDef2Me (
                    const void* src,
                          void* dst,
                       uint32_t idx
                  );


maxim_status_t MAXIM_Enable();

maxim_status_t MAXIM_Disable();


void MAXIM_InstallCallback( maxim_callback_t callback );

/**
 * testing HR sensor
 */
void MAXIM_Test();
