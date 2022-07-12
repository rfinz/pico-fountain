/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"


#define ADC_VREF 4.93
#define ADC_RANGE (1 << 12)
#define ADC_CONVERT (ADC_VREF / (ADC_RANGE - 1))

static uint8_t ADDRESS = 0x60;
static uint AUDIO_GATE_PIN = 27;
static uint AUDIO_ENVELOPE_PIN = 26;
static uint PUMP_START = 0x030;
static uint PUMP_MAINTAIN = 0x100;

void mcp4725_init();
void write_to_dac(uint val);


int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  uint pump_val;
  uint adc_raw;

  /// INITIALIZE STDIO FOR DEBUGGING ///
  //stdio_init_all();
  //printf("Beep boop, listening...\n");


  /// INITIALIZE I2C for pump level control ///
  i2c_init(i2c_default, 100 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  /// INITIALIZE DAC ///
  mcp4725_init();

  /// INITIALIZE ADC ///
  adc_init();

  /// INITIALIZE GPIO LED BLINK ///
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  /// INITIALIZE AUDIO SENSOR ///
  gpio_init(AUDIO_GATE_PIN);
  adc_gpio_init(AUDIO_ENVELOPE_PIN);
  adc_select_input(0);
  

  write_to_dac(PUMP_START);
  sleep_ms(5000);
  
  
  /// MAIN LOOP ///
  while (true) {
    
    sleep_ms(3);
    
    if(gpio_get(AUDIO_GATE_PIN)){
      gpio_put(LED_PIN, 1);
    } else {
      gpio_put(LED_PIN, 0);
    }

    adc_raw = adc_read();
    pump_val = PUMP_MAINTAIN + (adc_raw & 0x0FFF)/3;
    if((pump_val & 0xF000) >= 1) pump_val = 0xFFF;
    
    write_to_dac(pump_val);


    //printf("%d, dac: 0x%04x\n", (adc_raw & 0x0FFF)/4, pump_val);
    //sleep_ms(1000);
    
  }

}


void mcp4725_init(){
  uint8_t buf[3];
  buf[0] = (0x02 << 5) & 0xE6; // write, don't power down
  buf[1] = 0xFF; // full power
  buf[2] = 0xFF & 0xF0; // don't care about LSB for now
  
  i2c_write_blocking(i2c_default, ADDRESS, buf, 3, false);

}

void write_to_dac(uint val){
  uint8_t buf[3];
  buf[0] = (0x02 << 5) & 0xE6; // write, don't power down
  buf[1] = (val >> 4) & 0xFF; // take MSB
  buf[2] = val & 0xF0; // MSB are dropped by conversion to uint8_t
  
  i2c_write_blocking(i2c_default, ADDRESS, buf, 3, false);
}
