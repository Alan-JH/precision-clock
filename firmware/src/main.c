#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"

#include "clock.h"

#define PPS_GPIO 2
#define DISPBUS_LSB 8
#define DISPBUS_SIZE 12
#define DISPBUS_BITS 0xFFF
#define GPS_PPS_TIMEOUT 10 // Time that clock switches from GPS to RTC mode, in seconds, since last PPS rising edge
#define CLK_TUNING_STEP 1 // Proportional factor to tune local clock
#define RTC_UPDATE_RATE 250 // update rate in ms

// Display function defs
void clock_disp_update_isr();
void update_clock_font(Time t, bool ms_enable);

// RTC function defs
void init_i2c();
void set_rtc_all(Time time);
void read_rtc(Time * time);

bool gps_active; // True: GPS ACTIVE | False: RTC TIME

uint16_t millisecond_update_rate; // 1000 by default, increase to slow clock down, decrease to speed clock up

Time localclk; // Current time
Time last_pps; // Time of last PPS rising edge

void pps_isr()
{
    gpio_acknowledge_irq(PPS_GPIO, GPIO_IRQ_EDGE_RISE); // Ack interrupt
    
    if (localclk.milliseconds) // If milliseconds isnt zero upon PPS
    {
        if (localclk.milliseconds < 500) // If milliseconds is less than 500, assume wraparound and clock is running slightly fast
        {
            millisecond_update_rate -= CLK_TUNING_STEP; // TODO: Determine if you want proportional control
            localclk.milliseconds = 0; // Correct milliseconds
        }
        else // If milliseconds is 501 to 999, then clock is running slow
        {
            millisecond_update_rate += CLK_TUNING_STEP;
            localclk.milliseconds = 0; // Wrap to next second
            localclk.seconds ++;
        }
    }
    
    memcpy(&last_pps, &localclk, sizeof(Time)); // Copy new current time to PPS

    gps_active = 1; // Switch back to GPS mode in case it was previously in RTC mode
    uint64_t target = timer0_hw->timerawl + 1000000 * GPS_PPS_TIMEOUT;
    timer0_hw->alarm[2] = (uint32_t) target; // Set new GPS PPS Timeout
}

void gps_timeout_isr()
{
    hw_clear_bits(&timer0_hw->intr, 1 << 2);
    gps_active = 0;
}

void clock_rtc_update_isr()
{
    hw_clear_bits(&timer0_hw->intr, 1 << 3);
    if (gps_active)
        set_rtc_all(localclk); // Update RTC every second if GPS active
    else
        read_rtc(&localclk); // else update time from RTC
    
    uint64_t target = timer0_hw->timerawl + RTC_UPDATE_RATE * 1000;
    timer0_hw->alarm[3] = (uint32_t) target;
}

void clock_ms_update_isr()
{
    // TODO: Consider using a PWM slice counter peripheral instead of timer alarms for less jitter/more accuracy
    hw_clear_bits(&timer0_hw->intr, 1 << 0);
    uint64_t target = timer0_hw->timerawl + millisecond_update_rate; // t + 1ms
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
            }
        }
    }
    
    update_clock_font(localclk, gps_active);
}

void timer_isr(void) {
    uint32_t status = timer_hw->intr;

    if (status & (1 << 0)) clock_ms_update_isr();
    if (status & (1 << 1)) clock_disp_update_isr();
    if (status & (1 << 2)) gps_timeout_isr();
    if (status & (1 << 3)) clock_rtc_update_isr();
}

void init_timers()
{
    // Enable the interrupt for alarms
    hw_set_bits(&timer0_hw->inte, 0xf);
    // Set irq handler for alarm irq 0 and 1
    irq_set_exclusive_handler(TIMER0_IRQ_0, timer_isr);
    // Enable the alarm irqs
    irq_set_enabled(TIMER0_IRQ_0, 1);
    
    // Set timer 0 trigger time - 1ms
    uint64_t target_0 = timer0_hw->timerawl + millisecond_update_rate;
    // Set timer 1 trigger time - 250us - 4 updates per ms
    uint64_t target_1 = timer0_hw->timerawl + 10;

    uint64_t target_3 = timer0_hw->timerawl + RTC_UPDATE_RATE * 1000;

    // Write the lower 32 bits of the target time to the alarm which
    // will arm it
    timer0_hw->alarm[0] = (uint32_t) target_0;
    timer0_hw->alarm[1] = (uint32_t) target_1;
    timer0_hw->alarm[3] = (uint32_t) target_3;
}

void init_gpio()
{
    // Set directions to output and default to zero
    sio_hw->gpio_oe_set = DISPBUS_BITS << DISPBUS_LSB;
    sio_hw->gpio_clr = DISPBUS_BITS << DISPBUS_LSB;
    // Input enable on, output disable off
    for (int i = DISPBUS_LSB; i < DISPBUS_LSB + DISPBUS_SIZE; i++)
    {
        hw_write_masked(&pads_bank0_hw->io[i], PADS_BANK0_GPIO0_IE_BITS, PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);
        io_bank0_hw->io[i].ctrl = GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
        #if HAS_PADS_BANK0_ISOLATION
            // Remove pad isolation now that the correct peripheral is in control of the pad
            hw_clear_bits(&pads_bank0_hw->io[i], PADS_BANK0_GPIO0_ISO_BITS);
        #endif
    }

    // fill in
    sio_hw->gpio_oe_clr = (1 << PPS_GPIO);
    // Input enable on, output disable off
    hw_write_masked(&pads_bank0_hw->io[PPS_GPIO], PADS_BANK0_GPIO0_IE_BITS, PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);
    // Set GPIO functions to SIO
    io_bank0_hw->io[PPS_GPIO].ctrl = GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    #if HAS_PADS_BANK0_ISOLATION
        // Remove pad isolation now that the correct peripheral is in control of the pad
        hw_clear_bits(&pads_bank0_hw->io[PPS_GPIO], PADS_BANK0_GPIO0_ISO_BITS);
    #endif

    // Add handlers
    gpio_add_raw_irq_handler_masked((1 << PPS_GPIO), pps_isr);
    // Enable rising edge interrupts
    gpio_set_irq_enabled(PPS_GPIO, GPIO_IRQ_EDGE_RISE, 1);
    // Enable bank0 IRQ
    irq_set_enabled(IO_IRQ_BANK0, 1);
}

void clock_init()
{
    printf("Initializing Clock\n");

    init_gpio();
    init_i2c();

    millisecond_update_rate = 1000;
    gps_active = 0; // Default to GPS off and RTC mode

    read_rtc(&localclk);

    update_clock_font(localclk, gps_active);

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
