/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"

#include "pico/sem.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "ws2812.pio.h"


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




#define FRAC_BITS 4
#define NUM_PIXELS 64
#define WS2812_PIN_BASE 2

// horrible temporary hack to avoid changing pattern code
static uint8_t *current_string_out;
static bool current_string_4color;

static inline void put_pixel(uint32_t pixel_grb) {
    *current_string_out++ = pixel_grb & 0xffu;
    *current_string_out++ = (pixel_grb >> 8u) & 0xffu;
    *current_string_out++ = (pixel_grb >> 16u) & 0xffu;
    if (current_string_4color) {
        *current_string_out++ = 0; // todo adjust?
    }
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

void pattern_sparkle(uint len, uint t) {
    if (t % 8)
        return;
    for (int i = 0; i < len; ++i)
        put_pixel(rand() % 16 ? 0 : 0xffffffff);
}


typedef void (*pattern)(uint len, uint t);
const struct {
    pattern pat;
    const char *name;
} pattern_table[] = {
        {pattern_sparkle, "Sparkles"}
};

#define VALUE_PLANE_COUNT (8 + FRAC_BITS)
// we store value (8 bits + fractional bits of a single color (R/G/B/W) value) for multiple
// strings, in bit planes. bit plane N has the Nth bit of each string.
typedef struct {
    // stored MSB first
    uint32_t planes[VALUE_PLANE_COUNT];
} value_bits_t;

// Add FRAC_BITS planes of e to s and store in d
void add_error(value_bits_t *d, const value_bits_t *s, const value_bits_t *e) {
    uint32_t carry_plane = 0;
    // add the FRAC_BITS low planes
    for (int p = VALUE_PLANE_COUNT - 1; p >= 8; p--) {
        uint32_t e_plane = e->planes[p];
        uint32_t s_plane = s->planes[p];
        d->planes[p] = (e_plane ^ s_plane) ^ carry_plane;
        carry_plane = (e_plane & s_plane) | (carry_plane & (s_plane ^ e_plane));
    }
    // then just ripple carry through the non fractional bits
    for (int p = 7; p >= 0; p--) {
        uint32_t s_plane = s->planes[p];
        d->planes[p] = s_plane ^ carry_plane;
        carry_plane &= s_plane;
    }
}

typedef struct {
    uint8_t *data;
    uint data_len;
    uint frac_brightness; // 256 = *1.0;
} string_t;

// takes 8 bit color values, multiply by brightness and store in bit planes
void transform_strings(string_t **strings, uint num_strings, value_bits_t *values, uint value_length,
                       uint frac_brightness) {
    for (uint v = 0; v < value_length; v++) {
        memset(&values[v], 0, sizeof(values[v]));
        for (int i = 0; i < num_strings; i++) {
            if (v < strings[i]->data_len) {
                // todo clamp?
                uint32_t value = (strings[i]->data[v] * strings[i]->frac_brightness) >> 8u;
                value = (value * frac_brightness) >> 8u;
                for (int j = 0; j < VALUE_PLANE_COUNT && value; j++, value >>= 1u) {
                    if (value & 1u) values[v].planes[VALUE_PLANE_COUNT - 1 - j] |= 1u << i;
                }
            }
        }
    }
}

void dither_values(const value_bits_t *colors, value_bits_t *state, const value_bits_t *old_state, uint value_length) {
    for (uint i = 0; i < value_length; i++) {
        add_error(state + i, colors + i, old_state + i);
    }
}

// requested colors * 4 to allow for RGBW
static value_bits_t colors[NUM_PIXELS * 4];
// double buffer the state of the string, since we update next version in parallel with DMAing out old version
static value_bits_t states[2][NUM_PIXELS * 4];

// example - string 0 is RGB only
static uint8_t string0_data[NUM_PIXELS * 3];
// example - string 1 is RGBW
static uint8_t string1_data[NUM_PIXELS * 4];

string_t string0 = {
        .data = string0_data,
        .data_len = sizeof(string0_data),
        .frac_brightness = 0x40,
};

string_t string1 = {
        .data = string1_data,
        .data_len = sizeof(string1_data),
        .frac_brightness = 0x100,
};

string_t *strings[] = {
        &string0,
        &string1,
};

// bit plane content dma channel
#define DMA_CHANNEL 0
// chain channel for configuring main dma channel to output from disjoint 8 word fragments of memory
#define DMA_CB_CHANNEL 1

#define DMA_CHANNEL_MASK (1u << DMA_CHANNEL)
#define DMA_CB_CHANNEL_MASK (1u << DMA_CB_CHANNEL)
#define DMA_CHANNELS_MASK (DMA_CHANNEL_MASK | DMA_CB_CHANNEL_MASK)

// start of each value fragment (+1 for NULL terminator)
static uintptr_t fragment_start[NUM_PIXELS * 4 + 1];

// posted when it is safe to output a new set of values
static struct semaphore reset_delay_complete_sem;
// alarm handle for handling delay
alarm_id_t reset_delay_alarm_id;

int64_t reset_delay_complete(alarm_id_t id, void *user_data) {
    reset_delay_alarm_id = 0;
    sem_release(&reset_delay_complete_sem);
    // no repeat
    return 0;
}

void __isr dma_complete_handler() {
    if (dma_hw->ints0 & DMA_CHANNEL_MASK) {
        // clear IRQ
        dma_hw->ints0 = DMA_CHANNEL_MASK;
        // when the dma is complete we start the reset delay timer
        if (reset_delay_alarm_id) cancel_alarm(reset_delay_alarm_id);
        reset_delay_alarm_id = add_alarm_in_us(400, reset_delay_complete, NULL, true);
    }
}

void dma_init(PIO pio, uint sm) {
    dma_claim_mask(DMA_CHANNELS_MASK);

    // main DMA channel outputs 8 word fragments, and then chains back to the chain channel
    dma_channel_config channel_config = dma_channel_get_default_config(DMA_CHANNEL);
    channel_config_set_dreq(&channel_config, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&channel_config, DMA_CB_CHANNEL);
    channel_config_set_irq_quiet(&channel_config, true);
    dma_channel_configure(DMA_CHANNEL,
                          &channel_config,
                          &pio->txf[sm],
                          NULL, // set by chain
                          8, // 8 words for 8 bit planes
                          false);

    // chain channel sends single word pointer to start of fragment each time
    dma_channel_config chain_config = dma_channel_get_default_config(DMA_CB_CHANNEL);
    dma_channel_configure(DMA_CB_CHANNEL,
                          &chain_config,
                          &dma_channel_hw_addr(
                                  DMA_CHANNEL)->al3_read_addr_trig,  // ch DMA config (target "ring" buffer size 4) - this is (read_addr trigger)
                          NULL, // set later
                          1,
                          false);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);
    dma_channel_set_irq0_enabled(DMA_CHANNEL, true);
    irq_set_enabled(DMA_IRQ_0, true);
}

void output_strings_dma(value_bits_t *bits, uint value_length) {
    for (uint i = 0; i < value_length; i++) {
        fragment_start[i] = (uintptr_t) bits[i].planes; // MSB first
    }
    fragment_start[value_length] = 0;
    dma_channel_hw_addr(DMA_CB_CHANNEL)->al3_read_addr_trig = (uintptr_t) fragment_start;
}











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
  

  /// INITIALIZE WS2812 PARALLEL ///
  PIO pio = pio0;
  int sm = 0;
  
  uint offset = pio_add_program(pio, &ws2812_parallel_program);
  ws2812_parallel_program_init(pio, sm, offset, WS2812_PIN_BASE, count_of(strings), 800000);
  
  sem_init(&reset_delay_complete_sem, 1, 1); // initially posted so we don't block first time
  dma_init(pio, sm);
  int t = 0;


  
  /// MAIN LOOP ///
  while (true) {
    
    int pat = rand() % count_of(pattern_table);
    int dir = (rand() >> 30) & 1 ? 1 : -1;
    if (rand() & 1) dir = 0;
    
    if(gpio_get(AUDIO_GATE_PIN)){
      gpio_put(LED_PIN, 1);
    } else {
      gpio_put(LED_PIN, 0);
    }

    adc_raw = adc_read();
    pump_val = PUMP_MAINTAIN + (adc_raw & 0x0FFF)/3;
    if((pump_val & 0xF000) >= 1) pump_val = 0xFFF;
    
    write_to_dac(pump_val);

    int brightness = 0;
    uint current = 0;
    for (int i = 0; i < 1000; ++i) {
      current_string_out = string0.data;
      current_string_4color = false;
      pattern_table[pat].pat(NUM_PIXELS, t);
      current_string_out = string1.data;
      current_string_4color = true;
      pattern_table[pat].pat(NUM_PIXELS, t);
      
      transform_strings(strings, count_of(strings), colors, NUM_PIXELS * 4, brightness);
      dither_values(colors, states[current], states[current ^ 1], NUM_PIXELS * 4);
      sem_acquire_blocking(&reset_delay_complete_sem);
      output_strings_dma(states[current], NUM_PIXELS * 4);

      current ^= 1;
      t += dir;
      brightness++;
      if (brightness == (0x20 << FRAC_BITS)) brightness = 0;
    }
    memset(&states, 0, sizeof(states)); // clear out errors

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
