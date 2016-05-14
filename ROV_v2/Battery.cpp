#include <Arduino.h>
#include "Battery.h"

volatile int battery_percent = 0;
volatile int battery_sum = 0;
volatile int battery_count = 0;

/* Get Battery Level */
uint8_t get_battery_level() {
  int percent, analog_input;
  float input_voltage, max_vout, min_vout;
  
  max_vout = MAX_VOLTAGE * (R2 / (R1 + R2));
  min_vout = MIN_VOLTAGE * (R2 / (R1 + R2));
 
  /* Read Voltage Divider Input */
  analog_input = analogRead(BATTERY_PIN);
  
  /* Convert Analog Values to Voltage */
  input_voltage = analog_input * (max_vout / MAX_ANALOG);
  
  if (input_voltage < min_vout) {
    input_voltage = min_vout;
  }
  
  /* Calculate Battery Percentage */
  percent = (input_voltage - min_vout) * (MAX_PERCENT / (max_vout - min_vout));
  
  /* Keep Track of Samples for Running Average */
  battery_sum += percent;
  battery_count++;

  /* Calculate Battery Percentage Over Sample Period */
  if (battery_count == BATTERY_SAMPLES) {
    percent = battery_sum / battery_count;
    battery_percent = percent;
    battery_sum = 0;
    battery_count = 0;
  }
  
  return (uint8_t)battery_percent;
}
