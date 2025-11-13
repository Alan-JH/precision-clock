#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

#include "clock.h"

static char disp[16] = { // Display raw
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
static uint8_t digit; // Current digit being displayed
extern char font[]; // Font mapping for 7-segment display

Time localclk;

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

void clock_ms_update_isr()
{
    // TODO: Consider using a PWM slice counter peripheral instead of timer alarms for less jitter/more accuracy
    hw_clear_bits(&timer0_hw->intr, 1 << 0);
    uint64_t target = timer0_hw->timerawl + 1000; // t + 1ms
    timer0_hw->alarm[0] = (uint32_t) target; // Set new alarm immediately after clearing flag to reduce jitter
    if (++localclk.milliseconds > 999)
    {
        localclk.milliseconds = 0;
        if (++localclk.seconds > 59)
        {
            localclk.seconds = 0;
            if (++localclk.minutes > 59)
            {
                localclk.minutes = 0;
                if (++localclk.hours > 23)
                    localclk.hours = 0;
                disp[6] = font['0' + (localclk.hours / 10)];
                disp[7] = font['0' + (localclk.hours % 10)];
            }
            disp[8] = font['0' + (localclk.minutes / 10)];
            disp[9] = font['0' + (localclk.minutes % 10)];
        }
        disp[10] = font['0' + (localclk.seconds / 10)];
        disp[11] = font['0' + (localclk.seconds % 10)];
    }
    disp[12] = font['0' + (localclk.milliseconds / 100)];
    disp[13] = font['0' + ((localclk.milliseconds / 10) % 10)];
    disp[14] = font['0' + (localclk.milliseconds % 10)];
    
    return;
}

void init_timers()
{
    // Enable the interrupt for alarm 0 and 1
    hw_set_bits(&timer0_hw->inte, 0b11);
    // Set irq handler for alarm irq 0 and 1
    irq_set_exclusive_handler(TIMER0_IRQ_0, clock_ms_update_isr);
    irq_set_exclusive_handler(TIMER0_IRQ_1, clock_disp_update_isr);
    // Enable the alarm irqs
    irq_set_enabled(TIMER0_IRQ_0, 1);
    irq_set_enabled(TIMER0_IRQ_1, 1);
    
    // Set timer 0 trigger time - 1ms
    uint64_t target_0 = timer0_hw->timerawl + 1000;
    // Set timer 1 trigger time - 250us - 4 updates per ms
    uint64_t target_1 = timer0_hw->timerawl + 10;
    // Write the lower 32 bits of the target time to the alarm which
    // will arm it
    timer0_hw->alarm[0] = (uint32_t) target_0;
    timer0_hw->alarm[1] = (uint32_t) target_1;
}

void clock_init()
{
    // Set directions to output and default to zero
    sio_hw->gpio_oe_set = 0x0fff << 8;
    sio_hw->gpio_clr = 0x0fff << 8;
    // Input enable on, output disable off
    for (int i = 8; i < 20; i++)
    {
        hw_write_masked(&pads_bank0_hw->io[i], PADS_BANK0_GPIO0_IE_BITS, PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);
        io_bank0_hw->io[i].ctrl = GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
        #if HAS_PADS_BANK0_ISOLATION
            // Remove pad isolation now that the correct peripheral is in control of the pad
            hw_clear_bits(&pads_bank0_hw->io[i], PADS_BANK0_GPIO0_ISO_BITS);
        #endif
    }

    printf("initializing\n");


    localclk.milliseconds = 0;
    localclk.seconds = 0;
    localclk.minutes = 30;
    localclk.hours = 14;
    localclk.date = DEFAULT_DAY;
    localclk.month = DEFAULT_MONTH;
    localclk.year = DEFAULT_YEAR;

    disp[0] = font['0' + (localclk.month / 10)];
    disp[1] = font['0' + (localclk.month % 10)];
    disp[2] = font['0' + (localclk.date / 10)];
    disp[3] = font['0' + (localclk.date % 10)];
    disp[4] = font['0' + ((localclk.year / 10) % 10)];
    disp[5] = font['0' + (localclk.year % 10)];
    disp[6] = font['0' + (localclk.hours / 10)];
    disp[7] = font['0' + (localclk.hours % 10)];
    disp[8] = font['0' + (localclk.minutes / 10)];
    disp[9] = font['0' + (localclk.minutes % 10)];
    disp[10] = font['0' + (localclk.seconds / 10)];
    disp[11] = font['0' + (localclk.seconds % 10)];
    disp[12] = font['0' + (localclk.milliseconds / 100)];
    disp[13] = font['0' + ((localclk.milliseconds / 10) % 10)];
    disp[14] = font['0' + (localclk.milliseconds % 10)];

    digit = 0;

    init_timers(); // Do this after all long setup tasks as it starts the timers
}

int main()
{
    // Configures our microcontroller to 
    // communicate over UART through the TX/RX pins
    stdio_init_all();

    clock_init();

    
    for (;;);
    return 0;
}