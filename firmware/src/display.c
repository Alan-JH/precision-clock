#include <stdio.h>
#include <stdbool.h>
#include "clock.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

char disp[16] = { // Display raw
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
extern char font[];
static uint8_t digit; // Current digit being displayed

void clock_disp_update_isr()
{
    hw_clear_bits(&timer0_hw->intr, 1 << 1);
    
    sio_hw->gpio_clr = 0xfff << 8;
    sio_hw->gpio_set = ((digit << 8) | disp[digit]) << 8;
    digit ++;
    digit = digit & 0xf;

    uint64_t target = timer0_hw->timerawl + 10; // t + 10us
    timer0_hw->alarm[1] = (uint32_t) target; 
    return;
}

void update_clock_font(Time t, bool ms_enable)
{
    disp[0] = font['0' + (t.month / 10)];
    disp[1] = font['0' + (t.month % 10)];
    disp[2] = font['0' + (t.date / 10)];
    disp[3] = font['0' + (t.date % 10)];
    disp[4] = font['0' + ((t.year / 10) % 10)];
    disp[5] = font['0' + (t.year % 10)];
    disp[6] = font['0' + (t.hours / 10)];
    disp[7] = font['0' + (t.hours % 10)];
    disp[8] = font['0' + (t.minutes / 10)];
    disp[9] = font['0' + (t.minutes % 10)];
    disp[10] = font['0' + (t.seconds / 10)];
    disp[11] = font['0' + (t.seconds % 10)];
    if (ms_enable)
    {
        disp[12] = font['0' + (t.milliseconds / 100)];
        disp[13] = font['0' + ((t.milliseconds / 10) % 10)];
        disp[14] = font['0' + (t.milliseconds % 10)];
    }
    else 
    {
        disp[12] = font['-'];
        disp[13] = font['-'];
        disp[14] = font['-'];
    }
}