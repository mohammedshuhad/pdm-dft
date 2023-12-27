#include "ledVisualization.h"

LedVisualEffects::LedVisualEffects(uint16_t n_mels_bin,
                                   uint16_t number_of_leds) {
  number_of_mel_bins = n_mels_bin;
  n_leds = number_of_leds;
  led_list[0] = new uint8_t[n_leds / 2];
  led_list[1] = new uint8_t[n_leds / 2];
  led_list[2] = new uint8_t[n_leds / 2];
  gain_exp_filter =
      new ExponentialDiscreteFilter(number_of_mel_bins, 0.001, 0.99);
  gaussian_filter_scroll_effect = new one_dimensional_gaussian_filter(0.2);
}

LedVisualEffects::~LedVisualEffects() {
  delete[] led_list[0];
  delete[] led_list[1];
  delete[] led_list[2];
  delete gain_exp_filter;
  delete gaussian_filter_scroll_effect;
}

void LedVisualEffects::mirror(uint8_t* addressable_leds) {
  for (int i = 0; i < n_leds / 2; i++) {
    // addressable_leds[n_leds / 2 + i].r = led_list[0][i];
    // addressable_leds[n_leds / 2 + i].g = led_list[1][i];
    // addressable_leds[n_leds / 2 + i].b = led_list[2][i];
    // addressable_leds[n_leds / 2 - i - 1].r = led_list[0][i];
    // addressable_leds[n_leds / 2 - i - 1].g = led_list[1][i];
    // addressable_leds[n_leds / 2 - i - 1].b = led_list[2][i];
  }
}

void LedVisualEffects::scroll_mel_audio_data(float* mel_audio_data,
                                             uint8_t* addressable_leds) {
  for (int i = 0; i < number_of_mel_bins; i++) {
    mel_audio_data[i] = mel_audio_data[i] * mel_audio_data[i];
  }

  gain_exp_filter->update_smoothed_value_without_past(mel_audio_data);
  for (int i = 0; i < number_of_mel_bins; i++)
    if (gain_exp_filter->get_filter_output_buffer()[i] > 0.0)
      mel_audio_data[i] /= (gain_exp_filter->get_filter_output_buffer()[i]);

  float red = 0.0, green = 0.0, blue = 0.0;

  for (int i = 0; i < number_of_mel_bins; i++)
    if (i < number_of_mel_bins / 3) {
      red = std::max(red, mel_audio_data[i]);
    } else if (i > number_of_mel_bins * 2 / 3) {
      blue = std::max(blue, mel_audio_data[i]);
    } else {
      green = std::max(green, mel_audio_data[i]);
    }

  for (int i = ((n_leds / 2) - 1); i > 0; i--) {
    led_list[0][i] = (led_list[0][i - 1] == 0) ? 0 : led_list[0][i - 1] - 1;
    led_list[1][i] = (led_list[1][i - 1] == 0) ? 0 : led_list[1][i - 1] - 1;
    led_list[2][i] = (led_list[2][i - 1] == 0) ? 0 : led_list[2][i - 1] - 1;
  }
  gaussian_filter_scroll_effect->apply_gaussian_kernel_on_data(led_list[0],
                                                               n_leds / 2);
  gaussian_filter_scroll_effect->apply_gaussian_kernel_on_data(led_list[1],
                                                               n_leds / 2);
  gaussian_filter_scroll_effect->apply_gaussian_kernel_on_data(led_list[2],
                                                               n_leds / 2);

  led_list[0][0] = 255 * red;
  led_list[1][0] = 255 * green;
  led_list[2][0] = 255 * blue;

  mirror(addressable_leds);
}