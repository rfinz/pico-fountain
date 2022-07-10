/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"


static uint8_t ADDRESS = 0x60;
static uint AUDIO_GATE_PIN = 27;
static uint AUDIO_ENVELOPE_PIN = 26;

void mcp4725_init();
void write_to_dac(uint8_t val);


int main() {
  const uint LED_PIN = PICO_DEFAULT_LED_PIN;
  uint8_t adc_val;


  /// INITIALIZE I2C for pump level control ///
  i2c_init(i2c_default, 100 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  // configure DAC
  mcp4725_init();

  // initialize ADC
  adc_init();

  /// INITIALIZE GPIO LED BLINK ///
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  /// INITIALIZE AUDIO SENSOR ///
  gpio_init(AUDIO_GATE_PIN);
  adc_gpio_init(AUDIO_ENVELOPE_PIN);
  adc_select_input(0);
  
  
  while (true) {
    
    sleep_us(1);
    
    if(gpio_get(AUDIO_GATE_PIN)){
      gpio_put(LED_PIN, 1);
    } else {
      gpio_put(LED_PIN, 0);
    }

    adc_val = (adc_read() & 0xFF00) >> 8;
    write_to_dac(adc_val);
    
  }

}


void mcp4725_init(){
  uint8_t buf[3];
  buf[0] = (0x02 << 5) & 0xE6; // write, don't power down
  buf[1] = 0xFF; // full power
  buf[2] = 0xFF & 0xF8; // don't care about LSB for now
  
  i2c_write_blocking(i2c_default, ADDRESS, buf, 3, false);

}

void write_to_dac(uint8_t val){
  uint8_t buf[3];
  buf[0] = (0x02 << 5) & 0xE6; // write, don't power down
  buf[1] = val & 0xFF;
  buf[2] = 0xFF & 0xF8;
  
  i2c_write_blocking(i2c_default, ADDRESS, buf, 3, false);
}
