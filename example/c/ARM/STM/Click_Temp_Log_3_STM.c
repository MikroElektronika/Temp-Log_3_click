/*
Example for Temp_Log_3 Click

    Date          : Dec 2018.
    Author        : Nemanja Medakovic

Test configuration STM32 :
    
    MCU              : STM32F107VCT6
    Dev. Board       : EasyMx PRO v7 for STM32
    ARM Compiler ver : v6.0.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes peripherals and pins.
- Application Initialization - Initializes I2C interface and performs a device configuration for properly working.
  Also sets the temperature limit to the desired values.
- Application Task - (code snippet) - First ensures that the minimum conversion time is met, and then reads the
  ambient temperature calculated to the Celsius degrees.
  Also checks the limit status and shows a message when some limit condition is met.
Note : The temperature range that can be measured or written is from -40 to +125 Celsius degrees.
The user can change the measured temperature resolution, but the range remains the same.
The limit temperature resolution is always a 10bit, or 0.25 Celsius degrees.
If user wants to enable the EEPROM Write Protection, the A0 pin on the device must be set to the high voltage level.

Additional Functions :

- floatCut - Makes that float values be rounded on two decimal places.
- logUnit - Writes a Celsius degrees symbol on uart terminal.
- checkLimitStatus - Checks the limit status for each temperature reading cycle and writes a message on uart terminal
  when some limit condition is met.

*/

#include "Click_Temp_Log_3_types.h"
#include "Click_Temp_Log_3_config.h"

T_TEMPLOG3_RETVAL retStatus;
T_TEMPLOG3_DEG temperature;
char text[ 50 ];

void floatCut()
{
    uint8_t count;
    uint8_t conCnt = 0;
    uint8_t conVar = 0;

    for (count = 0; count < 50; count++)
    {
        if (text[ count ] == '.')
        {
            conVar = 1;
        }
        if (conVar == 1)
        {
            conCnt++;
        }
        if (conCnt > 3)
        {
            if ((text[ count ] == 'e') || (conVar == 2))
            {
                text[ count - (conCnt - 4) ] = text[ count ];
                text[ count ] = 0;
                conVar = 2;
            }
            else
            {
                text[ count ] = 0;
            }
        }
    }
}

void logUnit()
{
    text[ 0 ] = ' ';
    text[ 1 ] = 176;
    text[ 2 ] = 'C';
    text[ 3 ] = 0;
    
    mikrobus_logWrite( text, _LOG_TEXT );
}

void checkLimitStatus()
{
    if ((retStatus & _TEMPLOG3_TCRIT_DETECT) != _TEMPLOG3_OK)
    {
        mikrobus_logWrite( "**  Critical temperature detection!  **", _LOG_LINE );
    }
    if ((retStatus & _TEMPLOG3_TUPPER_DETECT) != _TEMPLOG3_OK)
    {
        mikrobus_logWrite( "**  Ambient temperature is higher than upper limit temperature!  **", _LOG_LINE );
    }
    else if ((retStatus & _TEMPLOG3_TLOWER_DETECT) != _TEMPLOG3_OK)
    {
        mikrobus_logWrite( "**  Ambient temperature is lower than lower limit temperature!  **", _LOG_LINE );
    }
}

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_INT_PIN, _GPIO_INPUT );

    mikrobus_i2cInit( _MIKROBUS1, &_TEMPLOG3_I2C_CFG[0] );

    mikrobus_logInit( _LOG_USBUART_A, 9600 );
    mikrobus_logWrite( "*** Initializing... ***", _LOG_LINE );

    Delay_ms( 100 );
}

void applicationInit()
{
    templog3_i2cDriverInit( (T_TEMPLOG3_P)&_MIKROBUS1_GPIO, (T_TEMPLOG3_P)&_MIKROBUS1_I2C, _TEMPLOG3_SLAVE_ADDR_A0_L );
    Delay_ms( 500 );
    
    templog3_writeReg( _TEMPLOG3_CONFIG_REG, _TEMPLOG3_TLIMIT_HYST_0_DEG | _TEMPLOG3_CONT_CONV_MODE | _TEMPLOG3_EVENT_OUTPUT_EN | _TEMPLOG3_EVENT_TCRIT_ONLY | _TEMPLOG3_EVENT_POL_ACT_HIGH | _TEMPLOG3_EVENT_COMPARATOR_MODE );
    templog3_writeReg( _TEMPLOG3_RESOLUTION_REG, _TEMPLOG3_11BIT_RESOLUTION );
    templog3_setTemp( _TEMPLOG3_TEMP_CRITICAL_REG, 26.5 );
    templog3_setTemp( _TEMPLOG3_TEMP_UPPER_REG, 30 );
    templog3_setTemp( _TEMPLOG3_TEMP_LOWER_REG, -5 );
    Delay_ms( 200 );
    
    mikrobus_logWrite( "** Temp-Log 3 is initialized **", _LOG_LINE );
    mikrobus_logWrite( "", _LOG_LINE );
}

void applicationTask()
{
    templog3_waitConvDone();

    retStatus = templog3_getTemp( _TEMPLOG3_TEMP_AMBIENT_REG, &temperature );
    
    FloatToStr( temperature, text );
    floatCut();
    mikrobus_logWrite( "**  Ambient temperature is : ", _LOG_TEXT );
    mikrobus_logWrite( text, _LOG_TEXT );
    logUnit();
    mikrobus_logWrite( "  **", _LOG_LINE );
    
    checkLimitStatus();
    
    Delay_ms( 300 );
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
        applicationTask();
    }
}
