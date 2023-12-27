#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include "defines.h"
#include "filter.h"
class MelSpectogram {
 private:
  uint16_t number_of_mel_bins, number_of_audio_data_samples, audio_sample_rate;
  float min_frequency, max_frequency, minimum_volume_threshold;
  float *temp_audio_data;
  uint8_t power_of_two = 0;
  // Hamming Window
  float *hamming_window_coefficient = NULL;
  void computeHammingWindowCoefficients();

  // Mel Matrix
  float **mel_spec_mat = NULL;
  float hertzToMelScale(float f);
  float melToHertzScale(float m);
  void calculate_mel_spec_coeff(uint16_t num_mel_bands, float freq_min,
                                float freq_max, uint16_t num_fft_bands,
                                uint16_t sample_rate);

 public:
  // Constructor
  MelSpectogram(uint16_t number_of_samples, uint16_t n_number_of_mel_bins,
                float min_frequency, float max_frequency,
                uint16_t sampling_rate, float min_volume_threshold);
  // Destructor
  ~MelSpectogram();

  void calculatePowerOfTwoBasedOnNumberOfSamples();
  void applyHammingWindow(float *audio_data);
  void computeMelSpectogram(float *input_audio_data, float *mel_audio_data);

  void arrangeAudioBufferDataBasedOnBitReversedIndex(float *audio_data);
  void computeFFT(float *audio_data_real);
  void computeAmplitude(float *audio_data_real, float *audio_data_imaginary);
  bool findMinMaxFromAudioBuffer(float *audio_buffer,
                                 float *mel_spectogram_data, float *min_data,
                                 float *max_data);
  class ExponentialDiscreteFilter *mel_spectogram_gain,
      *mel_spectogram_smoothing;
};
