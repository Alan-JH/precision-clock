#include <stdio.h>

#define DEFAULT_DAY 1
#define DEFAULT_MONTH 1
#define DEFAULT_YEAR 2025

typedef struct board_time
{
    uint8_t month;
    uint8_t date;
    uint16_t year;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint16_t milliseconds;
} Time;