#ifndef __ds1302_H
#define __ds1302_H

#include "main.h"

//DS1302 regs
#define DS1302_SEC_ADD              0x80    //Second: the highest bit CH is a clock stop flag bit, if it is 1, it means that the clock is abnormal, the high 3 bits of the 7th bit are the tens of the second, and the low 4 bits are the ones of the second.
#define DS1302_MIN_ADD              0x82    //Minute: The highest digit is not used, the upper 3 digits of the remaining 7 digits are the ten digits of the minute, and the lower 4 digits are the ones digit of the minute.
#define DS1302_HR_ADD               0x84    //Hour: If bit7 is 1, it means 12-hour system, 0 means 24-hour system; bit6 is fixed at 0, bit5 in 12-hour format, 0 means morning, 1 means afternoon, bit7 and bit4 in 24-hour format Together they represent the tens of the hour, and under 12 hours, bit4 represents the tens of the hour, and the lower 4 bits represent the ones of the hour.
#define DS1302_DATE_ADD             0x86    //Date: The upper 2 bits are fixed to 0, bit5 and bit4 are the tens of the date, and the lower 4 bits are the ones of the date.
#define DS1302_MONTH_ADD            0x88    //Month: The upper 3 bits are fixed to 0, bit4 is the tens of the month, and the lower 4 bits are the ones of the month.
#define DS1302_DAY_ADD              0x8a    //Week: The upper 5 bits are fixed at 0, and the lower 3 bits represent the week.
#define DS1302_YEAR_ADD             0x8c    //Year: The upper 4 digits represent the tens of the year, and the lower 4 digits represent the ones of the year. Please note that 00-99 here refers to the years 2000-2099.
#define DS1302_CONTROL_ADD          0x8e    //Protection bit: The highest bit is a write protection bit. If this bit is 1, it is forbidden to write data to any other register or the 31-byte RAM. Therefore, this bit must be written to 0 before writing data.
#define DS1302_CHARGER_ADD          0x90  	//0b10010000
#define DS1302_CLKBURST_ADD         0xbe  	//brust regs

#define DS1302_RAM_0				0xc0
#define DS1302_RAM_SIZE 			32

//DS1302 pins
#define DS1302_PIN_PORT CLOCK_CLK_GPIO_Port
#define DS1302_PIN_CLK CLOCK_CLK_Pin
#define DS1302_PIN_CLK_PORT DS1302_PIN_PORT
#define DS1302_PIN_DAT CLOCK_DATA_Pin
#define DS1302_PIN_DAT_PORT DS1302_PIN_PORT
#define DS1302_PIN_RST CLOCK_nRST_Pin
#define DS1302_PIN_RST_PORT DS1302_PIN_PORT


#define BOOL uint8_t
#define TRUE 1
#define FALSE 0

/**
 * year: 2000 - 2099
 * month: 1 - 12
 * date: 1 - 31
 * day: 1 - 7
 * isHourClock24: 1/0
 * isAm: 1/0。
 * hour: (isHourClock24 = 1: 0 - 23) / (isHourClock24 = 0: 1 - 12)
 * min: 00 - 59
 * sec: 00 - 59
 */
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t day;
    BOOL isHourClock24;
    BOOL isAm;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
} Time_Struct;

void DS1302_Init();

void DS1302_SetTime_Struct(Time_Struct *timeStruct);
Time_Struct DS1302_ReadTime_Struct();
void DS1302_SetTime_Struct_Burst(Time_Struct *timeStruct);
Time_Struct DS1302_ReadTime_Struct_Burst();
void DS1302_RamSave(int addr, uint8_t data);
uint8_t DS1302_RamRead(int addr);

#endif //___ds1302_H

//======================================================================================================
