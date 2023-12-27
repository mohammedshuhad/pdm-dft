#include "melSpectogram.h"

MelSpectogram::MelSpectogram(uint16_t number_of_samples,
                             uint16_t n_number_of_mel_bins, float min_frequency,
                             float max_frequency, uint16_t sampling_rate,
                             float min_volume_threshold)
{
  number_of_audio_data_samples = number_of_samples;
  number_of_mel_bins = n_number_of_mel_bins;
  min_frequency = min_frequency;
  max_frequency = max_frequency;
  audio_sample_rate = sampling_rate;
  minimum_volume_threshold = min_volume_threshold;
  temp_audio_data =
      (float *)malloc(number_of_audio_data_samples * sizeof(float));

  // Calculate Hamming Coefficients, place them in a Lookuptable for dynamic use
  computeHammingWindowCoefficients();

  // Calculate Power Of Two based on number of samples
  calculatePowerOfTwoBasedOnNumberOfSamples();

  // Smoothing filters
  mel_spectogram_gain =
      new ExponentialDiscreteFilter(1, mel_spectogram_gain_alpha_decay, 0.99);
  mel_spectogram_smoothing = new ExponentialDiscreteFilter(
      number_of_mel_bins, mel_spectogram_smoothing_alpha_decay, 0.99);

  calculate_mel_spec_coeff(number_of_mel_bins, min_frequency, max_frequency,
                           number_of_audio_data_samples, audio_sample_rate);
}

MelSpectogram::~MelSpectogram()
{
  // Free resources
  free(temp_audio_data);
  if (hamming_window_coefficient)
    delete[] hamming_window_coefficient;
}

void MelSpectogram::calculatePowerOfTwoBasedOnNumberOfSamples()
{
  // For 1024, power_of_two would be 10
  // We increase power by One until the expression "numberOfSample shift right
  // by power_of_two Value" reaches One. For Ex,
  // for numberOfSamples = 8, and power_of_two = 0
  // (8 >> 0) & 1 = False, -> power_of_two: 1,
  // (8 >> 1) & 1 = False, -> power_of_two: 2,
  // (8 >> 2) & 1 = False, -> power_of_two: 3,
  // (8 >> 3) & 1 = True, -> breaks the while loop,

  while (((number_of_audio_data_samples >> power_of_two) & 1) != 1)
    power_of_two++;
}

bool MelSpectogram::findMinMaxFromAudioBuffer(float *audio_buffer,
                                              float *mel_spectogram_data,
                                              float *min_data,
                                              float *max_data)
{
  uint16_t i = 0;
  *min_data = audio_buffer[i];
  *max_data = audio_buffer[i];

  for (i = 1; i < number_of_audio_data_samples; i++)
  {
    if (audio_buffer[i] < *min_data)
      *min_data = audio_buffer[i];

    if (audio_buffer[i] > *max_data)
      *max_data = audio_buffer[i];

    if (*max_data - *min_data > minimum_volume_threshold)
      break;
  }

  if (i == number_of_audio_data_samples)
  {
    // Reset Mel Spectogram Data (no min or max found)
    for (i = 0; i < number_of_audio_data_samples; i++)
      mel_spectogram_data[i] = 0.0;

    return false;
  }
  else
  {
    return true;
  }
}

void MelSpectogram::computeMelSpectogram(float *input_audio_data,
                                         float *mel_audio_data)
{
  float min_audio_data, max_audio_data;
  float max_mel_spec_data;

  findMinMaxFromAudioBuffer(input_audio_data, mel_audio_data, &min_audio_data,
                            &max_audio_data);
  memcpy(temp_audio_data, input_audio_data,
         sizeof(float) * number_of_audio_data_samples);

  // Apply Smoothing to remove noise
  applyHammingWindow(temp_audio_data);

  // FFT Calculation
  computeFFT(temp_audio_data);

  max_mel_spec_data = 0.0;
  for (int i = 0; i < number_of_mel_bins; i++)
  {
    mel_audio_data[i] = 0.0;
    for (int j = 0; j < number_of_audio_data_samples / 2; j++)
    {
      mel_audio_data[i] += temp_audio_data[j] * mel_spec_mat[i][j];
    }
    mel_audio_data[i] = mel_audio_data[i] * mel_audio_data[i];
    max_mel_spec_data = std::max(mel_audio_data[i], max_mel_spec_data);
  }

  mel_spectogram_gain->update_smoothed_value(&max_mel_spec_data);

  if (max_mel_spec_data > 0.0)
  {
    for (int i = 0; i < number_of_mel_bins; i++)
    {
      mel_audio_data[i] /= max_mel_spec_data;
    }
  }
  mel_spectogram_smoothing->update_smoothed_value(mel_audio_data);
}
void MelSpectogram::arrangeAudioBufferDataBasedOnBitReversedIndex(
    float *audio_data)
{
  uint16_t j = 0;
  float tmp;
  for (uint16_t i = 0; i < (number_of_audio_data_samples - 1); i++)
  {
    if (i < j)
    {
      tmp = audio_data[i];
      audio_data[i] = audio_data[j];
      audio_data[j] = tmp;
    }
    uint16_t k = (number_of_audio_data_samples >> 1);
    while (k <= j)
    {
      j -= k;
      k >>= 1;
    }
    j += k;
  }
}

void MelSpectogram::computeAmplitude(float *audio_data_real,
                                     float *audio_data_imaginary)
{
  for (int i = 0; i < number_of_audio_data_samples; i++)
  {
    audio_data_real[i] =
        sqrt((audio_data_real[i] * audio_data_real[i]) +
             (audio_data_imaginary[i] * audio_data_imaginary[i]));
  }
}

// Calculate N Point DFT based on number_of_audio_data_samples
void MelSpectogram::computeFFT(float *audio_data_real)
{
  // temp container for Imaginary Data
  float *audio_data_imaginary;
  audio_data_imaginary = new float[number_of_audio_data_samples]();

  // Bit Reversal step
  arrangeAudioBufferDataBasedOnBitReversedIndex(audio_data_real);

  // Compute the FFT (Butterfly Method)
  float coeff_1 = -1.0;
  float coeff_2 = 0.0;
  uint16_t l2 = 1;
  uint16_t j = 0;
  for (uint8_t l = 0; l < power_of_two; l++)
  {
    uint16_t l1 = l2;
    l2 <<= 1;
    float u1 = 1.0;
    float u2 = 0.0;
    for (j = 0; j < l1; j++)
    {
      for (uint16_t i = j; i < number_of_audio_data_samples; i += l2)
      {
        uint16_t i1 = i + l1;
        float temp_var_1 =
            u1 * audio_data_real[i1] - u2 * audio_data_imaginary[i1];
        float temp_var_2 =
            u1 * audio_data_imaginary[i1] + u2 * audio_data_real[i1];
        audio_data_real[i1] = audio_data_real[i] - temp_var_1;
        audio_data_imaginary[i1] = audio_data_imaginary[i] - temp_var_2;
        audio_data_real[i] += temp_var_1;
        audio_data_imaginary[i] += temp_var_2;
      }
      float z = ((u1 * coeff_1) - (u2 * coeff_2));
      u2 = ((u1 * coeff_2) + (u2 * coeff_1));
      u1 = z;
    }
    coeff_2 = sqrt((1.0 - coeff_1) / 2.0);
    coeff_2 = -coeff_2;
    coeff_1 = sqrt((1.0 + coeff_1) / 2.0);
  }

  // We only care about amplitude
  computeAmplitude(audio_data_real, audio_data_imaginary);
  delete[] audio_data_imaginary;
}

float MelSpectogram::hertzToMelScale(float f)
{
  return 2595.0 * log10(1.0 + f / 700.0);
}

float MelSpectogram::melToHertzScale(float m)
{
  return 700.0 * (pow(10.0, m / 2595.0) - 1.0);
}

void MelSpectogram::calculate_mel_spec_coeff(uint16_t number_of_mel_spec_bins,
                                             float min_frequency,
                                             float max_frequecy,
                                             uint16_t number_of_fft_bins,
                                             uint16_t sample_rate)
{
  float low_freq_in_mel_scale = hertzToMelScale(min_frequency);
  float high_freq_in_mel_scale = hertzToMelScale(max_frequecy);

  mel_spec_mat = new float *[number_of_mel_spec_bins];
  for (uint16_t i = 0; i < number_of_mel_spec_bins; i++)
    mel_spec_mat[i] = new float[number_of_fft_bins];

  float *center_filter_frequency = new float[number_of_mel_spec_bins + 2];

  for (uint16_t i = 0; i < number_of_mel_spec_bins + 2; i++)
  {
    center_filter_frequency[i] =
        melToHertzScale(low_freq_in_mel_scale +
                        (high_freq_in_mel_scale - low_freq_in_mel_scale) /
                            (number_of_mel_spec_bins + 1) * i);
  }

  float *fft_bin_frequency = new float[number_of_fft_bins];

  for (uint16_t i = 0; i < number_of_fft_bins; i++)
  {
    fft_bin_frequency[i] = (sample_rate / 2.0 / (number_of_fft_bins - 1) * i);
  }

  for (uint16_t mel_bin = 1; mel_bin <= number_of_mel_spec_bins; mel_bin++)
  {
    for (uint16_t fft_bin = 0; fft_bin < number_of_fft_bins; fft_bin++)
    {
      float weight;
      if (fft_bin_frequency[fft_bin] < center_filter_frequency[mel_bin - 1])
      {
        weight = 0.0;
      }
      else if (fft_bin_frequency[fft_bin] <=
               center_filter_frequency[mel_bin])
      {
        weight = (fft_bin_frequency[fft_bin] -
                  center_filter_frequency[mel_bin - 1]) /
                 (center_filter_frequency[mel_bin] -
                  center_filter_frequency[mel_bin - 1]);
      }
      else if (fft_bin_frequency[fft_bin] <=
               center_filter_frequency[mel_bin + 1])
      {
        weight = (center_filter_frequency[mel_bin + 1] -
                  fft_bin_frequency[fft_bin]) /
                 (center_filter_frequency[mel_bin + 1] -
                  center_filter_frequency[mel_bin]);
      }
      else
      {
        weight = 0.0;
      }
      mel_spec_mat[mel_bin - 1][fft_bin] = weight;
    }
  }
  delete[] center_filter_frequency;
  delete[] fft_bin_frequency;
}

void MelSpectogram::computeHammingWindowCoefficients()
{
  hamming_window_coefficient = new float[number_of_audio_data_samples];

  // M: Number of output points in Hamming Window
  uint16_t M = (number_of_audio_data_samples - 1);

  // Hamming Window Formula: w[n] = 0.54 - 0.46 * cos (2*pi*n / (M))
  for (uint16_t i = 0; i < number_of_audio_data_samples; i++)
  {
    hamming_window_coefficient[i] = (0.54 - 0.46 * cos(2.0 * M_PI * i / M));
  }
}

/* multiply audio data buffer with Hamming Window (this is the discrete
version of it) */
void MelSpectogram::applyHammingWindow(float *audio_data)
{
  for (uint16_t i = 0; i < 0; i++)
  {
    audio_data[i] *= hamming_window_coefficient[i];
  }
}