/*
    __templog3_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __templog3_driver.h
@brief    Temp_Log_3 Driver
@mainpage Temp_Log_3 Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   TEMPLOG3
@brief      Temp_Log_3 Click Driver
@{

| Global Library Prefix | **TEMPLOG3** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Dec 2018.**      |
| Developer             | **Nemanja Medakovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _TEMPLOG3_H_
#define _TEMPLOG3_H_

/** 
 * @macro T_TEMPLOG3_P
 * @brief Driver Abstract type 
 */
#define T_TEMPLOG3_P    const uint8_t*
#define T_TEMPLOG3_RETVAL     uint8_t
#define T_TEMPLOG3_DEG        float

#define TEMP_RESOL  0.0625
#define SIGN_BIT    0x1000

/** @defgroup TEMPLOG3_COMPILE Compilation Config */              /** @{ */

//  #define   __TEMPLOG3_DRV_SPI__                            /**<     @macro __TEMPLOG3_DRV_SPI__  @brief SPI driver selector */
   #define   __TEMPLOG3_DRV_I2C__                            /**<     @macro __TEMPLOG3_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __TEMPLOG3_DRV_UART__                           /**<     @macro __TEMPLOG3_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup TEMPLOG3_VAR Variables */                           /** @{ */

/** Device Address */
extern const uint8_t _TEMPLOG3_SLAVE_ADDR_A0_H              ;
extern const uint8_t _TEMPLOG3_SLAVE_ADDR_A0_L              ;

/** Register Address */
extern const uint8_t _TEMPLOG3_CAPABILITY_REG               ;
extern const uint8_t _TEMPLOG3_CONFIG_REG                   ;
extern const uint8_t _TEMPLOG3_TEMP_UPPER_REG               ;
extern const uint8_t _TEMPLOG3_TEMP_LOWER_REG               ;
extern const uint8_t _TEMPLOG3_TEMP_CRITICAL_REG            ;
extern const uint8_t _TEMPLOG3_TEMP_AMBIENT_REG             ;
extern const uint8_t _TEMPLOG3_MANUFACT_ID_REG              ;
extern const uint8_t _TEMPLOG3_DEVICE_ID_REG                ;
extern const uint8_t _TEMPLOG3_RESOLUTION_REG               ;

/** Status mask for the Capability register */
extern const uint8_t _TEMPLOG3_EVENT_SHDN_STATUS_MASK       ;
extern const uint8_t _TEMPLOG3_I2C_TIMEOUT_STATUS_MASK      ;
extern const uint8_t _TEMPLOG3_HIGH_VOLT_INPUT_STATUS_MASK  ;
extern const uint8_t _TEMPLOG3_RESOLUTION_STATUS_MASK       ;
extern const uint8_t _TEMPLOG3_MEAS_RANGE_STATUS_MASK       ;
extern const uint8_t _TEMPLOG3_ACCURACY_STATUS_MASK         ;
extern const uint8_t _TEMPLOG3_ALARM_STATUS_MASK            ;

/** Settings for the Config register */
extern const uint16_t _TEMPLOG3_TLIMIT_HYST_0_DEG           ;
extern const uint16_t _TEMPLOG3_TLIMIT_HYST_ONE_HALF_DEG    ;
extern const uint16_t _TEMPLOG3_TLIMIT_HYST_3_DEG           ;
extern const uint16_t _TEMPLOG3_TLIMIT_HYST_6_DEG           ;
extern const uint16_t _TEMPLOG3_CONT_CONV_MODE              ;
extern const uint16_t _TEMPLOG3_SHUTDOWN_MODE               ;
extern const uint16_t _TEMPLOG3_TCRIT_LOCKED                ;
extern const uint16_t _TEMPLOG3_TUPPER_TLOWER_LOCKED        ;
extern const uint16_t _TEMPLOG3_INT_CLEAR                   ;
extern const uint16_t _TEMPLOG3_EVENT_OUTPUT_STATUS_MASK    ;
extern const uint16_t _TEMPLOG3_EVENT_OUTPUT_EN             ;
extern const uint16_t _TEMPLOG3_EVENT_ALL_TLIMIT            ;
extern const uint16_t _TEMPLOG3_EVENT_TCRIT_ONLY            ;
extern const uint16_t _TEMPLOG3_EVENT_POL_ACT_LOW           ;
extern const uint16_t _TEMPLOG3_EVENT_POL_ACT_HIGH          ;
extern const uint16_t _TEMPLOG3_EVENT_COMPARATOR_MODE       ;
extern const uint16_t _TEMPLOG3_EVENT_INTERRUPT_MODE        ;

/** Limit and function response possible status bytes */
extern const uint8_t _TEMPLOG3_TCRIT_DETECT                 ;
extern const uint8_t _TEMPLOG3_TUPPER_DETECT                ;
extern const uint8_t _TEMPLOG3_TLOWER_DETECT                ;
extern const uint8_t _TEMPLOG3_NBYTES_ERROR                 ;
extern const uint8_t _TEMPLOG3_TEMP_RANGE_ERROR             ;
extern const uint8_t _TEMPLOG3_ADDR_ERROR                   ;
extern const uint8_t _TEMPLOG3_ALARMING                     ;
extern const uint8_t _TEMPLOG3_OK                           ;

/** Settings for the Resolution register */
extern const uint8_t _TEMPLOG3_12BIT_RESOLUTION             ;
extern const uint8_t _TEMPLOG3_11BIT_RESOLUTION             ;
extern const uint8_t _TEMPLOG3_10BIT_RESOLUTION             ;
extern const uint8_t _TEMPLOG3_9BIT_RESOLUTION              ;

/** Settings for the EEPROM writing */
extern const uint8_t _TEMPLOG3_EEPROM_WRITE                 ;
extern const uint8_t _TEMPLOG3_SW_WRITE_PROTECT             ;
extern const uint8_t _TEMPLOG3_CLEAR_WRITE_PROTECT          ;

/** Maximal EEPROM size in bytes */
extern const uint16_t _TEMPLOG3_EEPROM_SIZE                 ;

                                                                       /** @} */
/** @defgroup TEMPLOG3_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup TEMPLOG3_INIT Driver Initialization */              /** @{ */

#ifdef   __TEMPLOG3_DRV_SPI__
void templog3_spiDriverInit(T_TEMPLOG3_P gpioObj, T_TEMPLOG3_P spiObj);
#endif
#ifdef   __TEMPLOG3_DRV_I2C__
void templog3_i2cDriverInit(T_TEMPLOG3_P gpioObj, T_TEMPLOG3_P i2cObj, uint8_t slave);
#endif
#ifdef   __TEMPLOG3_DRV_UART__
void templog3_uartDriverInit(T_TEMPLOG3_P gpioObj, T_TEMPLOG3_P uartObj);
#endif

                                                                       /** @} */
/** @defgroup TEMPLOG3_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Generic Write function
 *
 * @param[in] regAddr  Address where data be written
 * @param[in] dataIn  16bit data to be written
 *
 * @returns 0 - OK, 2 - Wrong address
 *
 * Function writes a 16bit data to the desired register.
 */
T_TEMPLOG3_RETVAL templog3_writeReg( uint8_t regAddr, uint16_t dataIn );

/**
 * @brief Generic Read function
 *
 * @param[in] regAddr  Address which from data be read
 * @param[out] dataOut  Memory where 16bit data be placed
 *
 * @returns 0 - OK, 2 - Wrong address
 *
 * Function reads a 16bit data from the desired register.
 */
T_TEMPLOG3_RETVAL templog3_readReg( uint8_t regAddr, uint16_t *dataOut );

/**
 * @brief Set Address Pointer function
 *
 * @param[in] regAddr  Address on which the internal address pointer be set
 *
 * @returns 0 - OK, 2 - Wrong address
 *
 * Function sets the internal address pointer on the desired register address.
 */
T_TEMPLOG3_RETVAL templog3_setAddrPtr( uint8_t regAddr );

/**
 * @brief Repeated Read function
 *
 * @param[out] dataOut  Memory where 16bit data be placed
 *
 * Function reads a 16bit data from the register on which the internal address pointer was last set.
 */
void templog3_repeatedRead( uint16_t *dataOut );

/**
 * @brief Get Temperature function
 *
 * @param[in] tempSel  Address of the desired temperature register
 * @param[out] tempOut  Returns a temperature value calculated to the Celsius degrees
 *
 * @returns 0 - OK, 2 - Wrong address
 *
 * Function gets a temperature value from the desired temperature register calculated to the Celsius degrees.
 */
T_TEMPLOG3_RETVAL templog3_getTemp( uint8_t tempSel, T_TEMPLOG3_DEG *tempOut );

/**
 * @brief Set Temperature function
 *
 * @param[in] tempSel  Address of the desired temperature register
 * @param[in] tempIn  Temperature value to be written calculated to the Celsius degrees
 *
 * @returns 0 - OK, 2 - Wrong address, 3 - Temperature value is out of range
 *
 * Function sets a desired temperature register on the desired value calculated to the Celsius degrees.
 */
T_TEMPLOG3_RETVAL templog3_setTemp( uint8_t tempSel, T_TEMPLOG3_DEG tempIn );

/**
 * @brief Alarm-Event Check function
 *
 * @returns 0 - OK, 1 - Alarming, when alarm (event) pin polarity is set to active high
 *
 * Function checks the alarm (event) pin state.
 */
T_TEMPLOG3_RETVAL templog3_checkAlarm( void );

/**
 * @brief Conversion Time function
 *
 * Function ensures that the minimum conversion time is met.
 */
void templog3_waitConvDone( void );

/**
 * @brief EEPROM Single Write function
 *
 * @param[in] regAddr  Memory address where one byte data be written
 * @param[in] dataIn  Data byte to be written
 * @param[in] eepromMode  0 - EEPROM Write, 1 - SW Write Protection, 2 - Clear Write Protection
 *
 * Function writes a one byte data to the EEPROM including/excluding a write protection.
 */
void templog3_eepromByteWrite( uint8_t regAddr, uint8_t dataIn, uint8_t eepromMode );

/**
 * @brief EEPROM Page Write function
 *
 * @param[in] regAddr  Memory start address which from a writing cycle be started
 * @param[in] dataIn  Data to be written, 16 bytes of data
 *
 * Function writes a 16 bytes of data to the selected EEPROM page.
 */
void templog3_eepromPageWrite( uint8_t regAddr, uint8_t *dataIn );

/**
 * @brief EEPROM Current Address Read function
 *
 * @param[out] currentAddress  Returns the address of the last word accessed internally incremented by 1
 *
 * Function returns the address of the last word accessed internally incremented by 1.
 */
void templog3_eepromCurrAddrRead( uint8_t *currentAddress );

/**
 * @brief EEPROM Single Read function
 *
 * @param[in] regAddr  Memory address which from data byte be read
 * @param[out] dataOut  Memory where data byte be placed
 *
 * Function reads a one byte data from the EEPROM.
 */
void templog3_eepromByteRead( uint8_t regAddr, uint8_t *dataOut );

/**
 * @brief EEPROM Sequential Read function
 *
 * @param[in] regAddr  Memory start address which from a reading cycle be started
 * @param[out] dataOut  Memory where data be placed
 * @param[in] numBytes  Number of bytes to be read
 *
 * @returns 0 - OK, 4 - Number of bytes is out of range
 *
 * Function reads a desired number of bytes from the EEPROM.
 * @note Sum of the memory address and number of bytes must not be higher than 256.
 */
T_TEMPLOG3_RETVAL templog3_eepromSequentialRead( uint8_t regAddr, uint8_t *dataOut, uint16_t numBytes );

                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Temp_Log_3_STM.c
    @example Click_Temp_Log_3_TIVA.c
    @example Click_Temp_Log_3_CEC.c
    @example Click_Temp_Log_3_KINETIS.c
    @example Click_Temp_Log_3_MSP.c
    @example Click_Temp_Log_3_PIC.c
    @example Click_Temp_Log_3_PIC32.c
    @example Click_Temp_Log_3_DSPIC.c
    @example Click_Temp_Log_3_AVR.c
    @example Click_Temp_Log_3_FT90x.c
    @example Click_Temp_Log_3_STM.mbas
    @example Click_Temp_Log_3_TIVA.mbas
    @example Click_Temp_Log_3_CEC.mbas
    @example Click_Temp_Log_3_KINETIS.mbas
    @example Click_Temp_Log_3_MSP.mbas
    @example Click_Temp_Log_3_PIC.mbas
    @example Click_Temp_Log_3_PIC32.mbas
    @example Click_Temp_Log_3_DSPIC.mbas
    @example Click_Temp_Log_3_AVR.mbas
    @example Click_Temp_Log_3_FT90x.mbas
    @example Click_Temp_Log_3_STM.mpas
    @example Click_Temp_Log_3_TIVA.mpas
    @example Click_Temp_Log_3_CEC.mpas
    @example Click_Temp_Log_3_KINETIS.mpas
    @example Click_Temp_Log_3_MSP.mpas
    @example Click_Temp_Log_3_PIC.mpas
    @example Click_Temp_Log_3_PIC32.mpas
    @example Click_Temp_Log_3_DSPIC.mpas
    @example Click_Temp_Log_3_AVR.mpas
    @example Click_Temp_Log_3_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __templog3_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */