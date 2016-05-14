#ifndef BATTERY_H
#define BATTERY_H

#define BATTERY_PIN 0
#define MIN_VOLTAGE 12.8f
#define MAX_VOLTAGE 16.8f
#define R1 1763.0f
#define R2 745.0f
#define MAX_ANALOG 1023
#define MAX_PERCENT 100.0f
#define BATTERY_SAMPLES 100

extern volatile int battery_percent;
extern volatile int battery_sum;
extern volatile int battery_count;

/* Get Battery Level */
uint8_t get_battery_level();

#endif