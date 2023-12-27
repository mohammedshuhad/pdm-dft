#pragma once

#include <cmath>

#include "filter.h"

class LedVisualEffects {
 private:
  class ExponentialDiscreteFilter *gain_exp_filter;
  class one_dimensional_gaussian_filter *gaussian_filter_scroll_effect;
  uint8_t *led_list[3];
  uint16_t number_of_mel_bins, n_leds;
  void mirror(uint8_t *physic_leds);

 public:
  LedVisualEffects(uint16_t number_of_mel_bins, uint16_t leds_num);
  void scroll_mel_audio_data(float *mel_data, uint8_t *physic_leds);
  ~LedVisualEffects();
};