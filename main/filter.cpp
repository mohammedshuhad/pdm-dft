#include "filter.h"

ExponentialDiscreteFilter::ExponentialDiscreteFilter(float *val,
                                                     int length_of_filter,
                                                     float decay_alpha,
                                                     float rise_alpha) {
  len_of_filter = length_of_filter;
  exp_alpha_decay = decay_alpha;
  exp_alpha_rise = rise_alpha;

  filter_output_buffer = new float[length_of_filter];
  for (int i = 0; i < len_of_filter; i++) {
    filter_output_buffer[i] = val[i];
  }
}

ExponentialDiscreteFilter::ExponentialDiscreteFilter(int length_of_filter,
                                                     float decay_alpha,
                                                     float rise_alpha) {
  len_of_filter = length_of_filter;
  exp_alpha_decay = decay_alpha;
  exp_alpha_rise = rise_alpha;

  filter_output_buffer = new float[length_of_filter];
  for (int i = 0; i < len_of_filter; i++) {
    filter_output_buffer[i] = 0.0;
  }
}

ExponentialDiscreteFilter::~ExponentialDiscreteFilter() {
  delete[] filter_output_buffer;
}

float *ExponentialDiscreteFilter::get_filter_output_buffer() {
  return filter_output_buffer;
}

void ExponentialDiscreteFilter::update_smoothed_value_without_past(
    float *data) {
  for (int i = 0; i < len_of_filter; i++) {
    if (data[i] > filter_output_buffer[i]) {
      filter_output_buffer[i] =
          data[i] * exp_alpha_rise +
          filter_output_buffer[i] * (1.0 - exp_alpha_rise);
    } else {
      filter_output_buffer[i] =
          data[i] * exp_alpha_decay +
          filter_output_buffer[i] * (1.0 - exp_alpha_decay);
    }
  }
}

void ExponentialDiscreteFilter::update_smoothed_value(float *data) {
  for (int i = 0; i < len_of_filter; i++) {
    if (data[i] > filter_output_buffer[i]) {
      filter_output_buffer[i] =
          data[i] * exp_alpha_rise +
          filter_output_buffer[i] * (1.0 - exp_alpha_rise);
    } else {
      filter_output_buffer[i] =
          data[i] * exp_alpha_decay +
          filter_output_buffer[i] * (1.0 - exp_alpha_decay);
    }
    data[i] = filter_output_buffer[i];
  }
}

void ExponentialDiscreteFilter::update_smoothed_value(uint8_t *data) {
  for (int i = 0; i < len_of_filter; i++) {
    if (data[i] > filter_output_buffer[i]) {
      filter_output_buffer[i] =
          data[i] * exp_alpha_rise +
          filter_output_buffer[i] * (1.0 - exp_alpha_rise);
    } else {
      filter_output_buffer[i] =
          data[i] * exp_alpha_decay +
          filter_output_buffer[i] * (1.0 - exp_alpha_decay);
    }
    data[i] = filter_output_buffer[i];
  }
}

one_dimensional_gaussian_filter::one_dimensional_gaussian_filter(float sigma) {
  float sum = 0;
  radius = (int)(4.0 * sigma + 0.5);
  gaussian_kernel_1d = new float[radius * 2 + 1];
  gaussian_kernel_1d[radius] = 1.0;
  for (int i = 1; i <= radius; i++) {
    gaussian_kernel_1d[radius + i] = exp(-0.5 / (sigma * sigma) * i * i);
  }

  for (int i = 1; i <= radius; i++) {
    sum += gaussian_kernel_1d[radius + i];
  }
  sum = 2.0 * sum + 1.0;

  gaussian_kernel_1d[radius] = 1.0 / sum;
  for (int i = 1; i <= radius; i++) {
    gaussian_kernel_1d[radius + i] /= sum;
    gaussian_kernel_1d[radius - i] = gaussian_kernel_1d[radius + i];
  }
}

one_dimensional_gaussian_filter::~one_dimensional_gaussian_filter() {
  delete[] gaussian_kernel_1d;
}

void one_dimensional_gaussian_filter::apply_gaussian_kernel_on_data(
    uint8_t *data, int num) {
  uint8_t *origin = new uint8_t[num];
  for (int i = 0; i < num; i++) {
    origin[i] = data[i];
  }

  for (int i = 0; i < num; i++) {
    data[i] = (uint8_t)(origin[i] * gaussian_kernel_1d[radius]);
    for (int j = 1; j <= radius; j++) {
      data[i] += (uint8_t)(origin[(i - j < 0) ? (j - i - 1) : (i - j)] *
                           gaussian_kernel_1d[radius - j]);
      data[i] +=
          (uint8_t)(origin[(i + j >= num) ? (2 * num - j - i - 1) : (i + j)] *
                    gaussian_kernel_1d[radius + j]);
    }
  }
  delete[] origin;
}

void one_dimensional_gaussian_filter::apply_gaussian_kernel_on_data(float *data,
                                                                    int num) {
  float *origin = new float[num];
  for (int i = 0; i < num; i++) {
    origin[i] = data[i];
  }

  for (int i = 0; i < num; i++) {
    data[i] = origin[i] * gaussian_kernel_1d[radius];
    for (int j = 1; j <= radius; j++) {
      data[i] += origin[(i - j < 0) ? (j - i - 1) : (i - j)] *
                 gaussian_kernel_1d[radius - j];
      data[i] += origin[(i + j >= num) ? (2 * num - j - i - 1) : (i + j)] *
                 gaussian_kernel_1d[radius + j];
    }
  }
  delete[] origin;
}
