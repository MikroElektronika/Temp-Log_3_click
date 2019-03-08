/*
    __templog3_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__templog3_driver.h"
#include "__templog3_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __TEMPLOG3_DRV_I2C__
static uint8_t _slaveAddress;
static uint8_t _slaveEEPROM;
#endif

static uint8_t resolution;
static uint8_t nBytes;

const uint8_t _TEMPLOG3_SLAVE_ADDR_A0_H                    = 0x19;
const uint8_t _TEMPLOG3_SLAVE_ADDR_A0_L                    = 0x18;

const uint8_t _TEMPLOG3_CAPABILITY_REG                     = 0x00;
const uint8_t _TEMPLOG3_CONFIG_REG                         = 0x01;
const uint8_t _TEMPLOG3_TEMP_UPPER_REG                     = 0x02;
const uint8_t _TEMPLOG3_TEMP_LOWER_REG                     = 0x03;
const uint8_t _TEMPLOG3_TEMP_CRITICAL_REG                  = 0x04;
const uint8_t _TEMPLOG3_TEMP_AMBIENT_REG                   = 0x05;
const uint8_t _TEMPLOG3_MANUFACT_ID_REG                    = 0x06;
const uint8_t _TEMPLOG3_DEVICE_ID_REG                      = 0x07;
const uint8_t _TEMPLOG3_RESOLUTION_REG                     = 0x08;

const uint8_t _TEMPLOG3_EVENT_SHDN_STATUS_MASK             = 0x80;
const uint8_t _TEMPLOG3_I2C_TIMEOUT_STATUS_MASK            = 0x40;
const uint8_t _TEMPLOG3_HIGH_VOLT_INPUT_STATUS_MASK        = 0x20;
const uint8_t _TEMPLOG3_RESOLUTION_STATUS_MASK             = 0x18;
const uint8_t _TEMPLOG3_MEAS_RANGE_STATUS_MASK             = 0x04;
const uint8_t _TEMPLOG3_ACCURACY_STATUS_MASK               = 0x02;
const uint8_t _TEMPLOG3_ALARM_STATUS_MASK                  = 0x01;

const uint16_t _TEMPLOG3_TLIMIT_HYST_0_DEG                 = 0x0000;
const uint16_t _TEMPLOG3_TLIMIT_HYST_ONE_HALF_DEG          = 0x0200;
const uint16_t _TEMPLOG3_TLIMIT_HYST_3_DEG                 = 0x0400;
const uint16_t _TEMPLOG3_TLIMIT_HYST_6_DEG                 = 0x0600;
const uint16_t _TEMPLOG3_CONT_CONV_MODE                    = 0x0000;
const uint16_t _TEMPLOG3_SHUTDOWN_MODE                     = 0x0100;
const uint16_t _TEMPLOG3_TCRIT_LOCKED                      = 0x0080;
const uint16_t _TEMPLOG3_TUPPER_TLOWER_LOCKED              = 0x0040;
const uint16_t _TEMPLOG3_INT_CLEAR                         = 0x0020;
const uint16_t _TEMPLOG3_EVENT_OUTPUT_STATUS_MASK          = 0x0010;
const uint16_t _TEMPLOG3_EVENT_OUTPUT_EN                   = 0x0008;
const uint16_t _TEMPLOG3_EVENT_ALL_TLIMIT                  = 0x0000;
const uint16_t _TEMPLOG3_EVENT_TCRIT_ONLY                  = 0x0004;
const uint16_t _TEMPLOG3_EVENT_POL_ACT_LOW                 = 0x0000;
const uint16_t _TEMPLOG3_EVENT_POL_ACT_HIGH                = 0x0002;
const uint16_t _TEMPLOG3_EVENT_COMPARATOR_MODE             = 0x0000;
const uint16_t _TEMPLOG3_EVENT_INTERRUPT_MODE              = 0x0001;

const uint8_t _TEMPLOG3_TCRIT_DETECT                       = 0x80;
const uint8_t _TEMPLOG3_TUPPER_DETECT                      = 0x40;
const uint8_t _TEMPLOG3_TLOWER_DETECT                      = 0x20;
const uint8_t _TEMPLOG3_NBYTES_ERROR                       = 0x04;
const uint8_t _TEMPLOG3_TEMP_RANGE_ERROR                   = 0x03;
const uint8_t _TEMPLOG3_ADDR_ERROR                         = 0x02;
const uint8_t _TEMPLOG3_ALARMING                           = 0x01;
const uint8_t _TEMPLOG3_OK                                 = 0x00;

const uint8_t _TEMPLOG3_12BIT_RESOLUTION                   = 0x03;
const uint8_t _TEMPLOG3_11BIT_RESOLUTION                   = 0x02;
const uint8_t _TEMPLOG3_10BIT_RESOLUTION                   = 0x01;
const uint8_t _TEMPLOG3_9BIT_RESOLUTION                    = 0x00;

const uint8_t _TEMPLOG3_EEPROM_WRITE                       = 0x00;
const uint8_t _TEMPLOG3_SW_WRITE_PROTECT                   = 0x01;
const uint8_t _TEMPLOG3_CLEAR_WRITE_PROTECT                = 0x02;

const uint16_t _TEMPLOG3_EEPROM_SIZE                       = 256;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __TEMPLOG3_DRV_SPI__

void templog3_spiDriverInit(T_TEMPLOG3_P gpioObj, T_TEMPLOG3_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __TEMPLOG3_DRV_I2C__

void templog3_i2cDriverInit(T_TEMPLOG3_P gpioObj, T_TEMPLOG3_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    _slaveEEPROM = 0x50 | (slave & 0x07);
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    resolution = _TEMPLOG3_10BIT_RESOLUTION;
    nBytes = 2;
}

#endif
#ifdef   __TEMPLOG3_DRV_UART__

void templog3_uartDriverInit(T_TEMPLOG3_P gpioObj, T_TEMPLOG3_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif

/* ----------------------------------------------------------- IMPLEMENTATION */

T_TEMPLOG3_RETVAL templog3_writeReg( uint8_t regAddr, uint16_t dataIn )
{
    uint8_t tempData[ 3 ];
    uint8_t numBytes;
    
    if ( (regAddr < 0x01) || ((regAddr > 0x04) && (regAddr != 0x08)) )
    {
        return _TEMPLOG3_ADDR_ERROR;
    }
    
    tempData[ 0 ] = regAddr;
    
    if (regAddr == _TEMPLOG3_RESOLUTION_REG)
    {
        tempData[ 1 ] = dataIn;
        numBytes = 2;
        nBytes = 1;
        resolution = dataIn & 0x03;
    }
    else
    {
        tempData[ 1 ] = dataIn >> 8;
        tempData[ 2 ] = dataIn;
        numBytes = 3;
        nBytes = 2;
    }
    
    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, tempData, numBytes, END_MODE_STOP );
    
    return _TEMPLOG3_OK;
}

T_TEMPLOG3_RETVAL templog3_readReg( uint8_t regAddr, uint16_t *dataOut )
{
    uint8_t tempData[ 2 ];
    uint16_t retVal;
    uint8_t numBytes;
    
    if (regAddr > 0x08)
    {
        return _TEMPLOG3_ADDR_ERROR;
    }
    
    if (regAddr == _TEMPLOG3_RESOLUTION_REG)
    {
        numBytes = 1;
        nBytes = 1;
    }
    else
    {
        numBytes = 2;
        nBytes = 2;
    }
    
    tempData[ 0 ] = regAddr;
    
    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, tempData, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveAddress, tempData, numBytes, END_MODE_STOP );
    
    retVal = tempData[ 0 ];
    
    if (regAddr != _TEMPLOG3_RESOLUTION_REG)
    {
        retVal <<= 8;
        retVal |= tempData[ 1 ];
    }
    *dataOut = retVal;
    
    return _TEMPLOG3_OK;
}

T_TEMPLOG3_RETVAL templog3_setAddrPtr( uint8_t regAddr )
{
    uint8_t registerAddr;
    
    if (regAddr > 0x08)
    {
        return _TEMPLOG3_ADDR_ERROR;
    }
    registerAddr = regAddr;
    
    if (regAddr == _TEMPLOG3_RESOLUTION_REG)
    {
        nBytes = 1;
    }
    else
    {
        nBytes = 2;
    }
    
    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, &registerAddr, 1, END_MODE_STOP );
    
    return _TEMPLOG3_OK;
}

void templog3_repeatedRead( uint16_t *dataOut )
{
    uint8_t tempData[ 2 ];
    uint16_t retVal;
    
    hal_i2cStart();
    hal_i2cRead( _slaveAddress, tempData, nBytes, END_MODE_STOP );
    
    retVal = tempData[ 0 ];
    
    if (nBytes == 2)
    {
        retVal <<= 8;
        retVal |= tempData[ 1 ];
    }
    *dataOut = retVal;
}

T_TEMPLOG3_RETVAL templog3_getTemp( uint8_t tempSel, T_TEMPLOG3_DEG *tempOut )
{
    T_TEMPLOG3_DEG tempRes;
    uint16_t temp;
    int16_t tempVal;
    T_TEMPLOG3_RETVAL limitStat;
    static uint8_t tempSelPrev = 0;
    
    if ((tempSel < 0x02) || (tempSel > 0x05))
    {
        return _TEMPLOG3_ADDR_ERROR;
    }
    
    if (tempSelPrev != tempSel)
    {
        templog3_setAddrPtr( tempSel );
        tempSelPrev = tempSel;
    }
    
    templog3_repeatedRead( &temp );
    
    limitStat = (temp & 0xE000) >> 8;
    
    if (temp & SIGN_BIT)
    {
        tempVal = temp | 0xE000;
    }
    else
    {
        tempVal = temp & 0x1FFF;
    }
    
    tempRes = tempVal * TEMP_RESOL;
    *tempOut = tempRes;
    
    return limitStat;
}

T_TEMPLOG3_RETVAL templog3_setTemp( uint8_t tempSel, T_TEMPLOG3_DEG tempIn )
{
    T_TEMPLOG3_DEG tempRes;
    int16_t tempVal;
    
    if ((tempSel < 0x02) || (tempSel > 0x04))
    {
        return _TEMPLOG3_ADDR_ERROR;
    }
    if ((tempIn < -40) || (tempIn > 125))
    {
        return _TEMPLOG3_TEMP_RANGE_ERROR;
    }
    
    tempRes = tempIn / TEMP_RESOL;
    tempVal = tempRes;
    tempVal &= 0x1FFC;
    
    templog3_writeReg( tempSel, tempVal );
    
    return _TEMPLOG3_OK;
}

T_TEMPLOG3_RETVAL templog3_checkAlarm( void )
{
    if (hal_gpio_intGet())
    {
        return _TEMPLOG3_ALARMING;
    }
    else
    {
        return _TEMPLOG3_OK;
    }
}

void templog3_waitConvDone( void )
{
    uint8_t timeCnt;
    uint8_t cnt;
    
    switch (resolution)
    {
        case _TEMPLOG3_9BIT_RESOLUTION :
        {
            timeCnt = 4;
        break;
        }
        case _TEMPLOG3_10BIT_RESOLUTION :
        {
            timeCnt = 7;
        break;
        }
        case _TEMPLOG3_11BIT_RESOLUTION :
        {
            timeCnt = 14;
        break;
        }
        case _TEMPLOG3_12BIT_RESOLUTION :
        {
            timeCnt = 27;
        break;
        }
        default :
        {
            timeCnt = 7;
        break;
        }
    }
    
    for (cnt = 0; cnt < timeCnt; cnt++)
    {
        Delay_10ms();
    }
}

void templog3_eepromByteWrite( uint8_t regAddr, uint8_t dataIn, uint8_t eepromMode )
{
    uint8_t tempData[ 2 ];
    uint8_t _slaveAddr;
    
    switch (eepromMode)
    {
        case _TEMPLOG3_EEPROM_WRITE :
        {
            _slaveAddr = _slaveEEPROM;
        break;
        }
        case _TEMPLOG3_SW_WRITE_PROTECT :
        {
            _slaveAddr = 0x31;
        break;
        }
        case _TEMPLOG3_CLEAR_WRITE_PROTECT :
        {
            _slaveAddr = 0x33;
        break;
        }
        default :
        {
            return;
        }
    }
    
    tempData[ 0 ] = regAddr;
    tempData[ 1 ] = dataIn;
    
    hal_i2cStart();
    hal_i2cWrite( _slaveAddr, tempData, 2, END_MODE_STOP );
}

void templog3_eepromPageWrite( uint8_t regAddr, uint8_t *dataIn )
{
    uint8_t tempData[ 17 ];
    uint8_t cnt;
    
    tempData[ 0 ] = regAddr;
    
    for (cnt = 1; cnt <= 16; cnt++)
    {
        tempData[ cnt ] = *dataIn;
        dataIn++;
    }
    
    hal_i2cStart();
    hal_i2cWrite( _slaveEEPROM, tempData, 17, END_MODE_STOP );
}

void templog3_eepromCurrAddrRead( uint8_t *currentAddress )
{
    hal_i2cStart();
    hal_i2cRead( _slaveEEPROM, currentAddress, 1, END_MODE_STOP );
}

void templog3_eepromByteRead( uint8_t regAddr, uint8_t *dataOut )
{
    uint8_t registerAddr = regAddr;
    
    hal_i2cStart();
    hal_i2cWrite( _slaveEEPROM, &registerAddr, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveEEPROM, dataOut, 1, END_MODE_STOP );
}

T_TEMPLOG3_RETVAL templog3_eepromSequentialRead( uint8_t regAddr, uint8_t *dataOut, uint16_t numBytes )
{
    uint8_t registerAddr = regAddr;
    
    if ((regAddr + numBytes) > _TEMPLOG3_EEPROM_SIZE)
    {
        return _TEMPLOG3_NBYTES_ERROR;
    }
    
    hal_i2cStart();
    hal_i2cWrite( _slaveEEPROM, &registerAddr, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveEEPROM, dataOut, numBytes, END_MODE_STOP );
    
    return _TEMPLOG3_OK;
}

/* -------------------------------------------------------------------------- */
/*
  __templog3_driver.c

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