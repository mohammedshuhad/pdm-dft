#pragma once

#include <cmath>

class ExponentialDiscreteFilter
{
private:
  float *filter_output_buffer;
  int len_of_filter;
  float exp_alpha_decay;
  float exp_alpha_rise;

public:
  ExponentialDiscreteFilter(float *val, int val_len, float alpha_decay,
                            float alpha_rise);
  ExponentialDiscreteFilter(int val_len, float alpha_decay, float alpha_rise);
  ~ExponentialDiscreteFilter();
  void update_smoothed_value(uint8_t *data);
  void update_smoothed_value_without_past(float *data);
  void update_smoothed_value(float *data);
  float *get_filter_output_buffer();
};

class one_dimensional_gaussian_filter
{
private:
  float *gaussian_kernel_1d;
  int radius;

public:
  one_dimensional_gaussian_filter(float sigma);
  ~one_dimensional_gaussian_filter();
  void apply_gaussian_kernel_on_data(float *data, int num);
  void apply_gaussian_kernel_on_data(uint8_t *data, int num);
};