#pragma once
#define audioBufferDataLength 64

const uint16_t BUFFER_SIZE = 1024;
const uint8_t N_BUFFER_ROLLING_HISTORY = 2;
const uint16_t SAMPLE_RATE = 44100;
const uint16_t NUMBER_OF_MEL_BIN = 18;
const float MIN_FREQUENCY = 200;
const float MAX_FREQUENCY = 12000;
const float MIN_VOLUME_THRESHOLD = 0.0003;

const uint16_t NUMBER_OF_LEDS = 42;
const int LED_STRIP_DATA_PIN = 21;

#define mel_spectogram_gain_alpha_decay 0.06
#define mel_spectogram_smoothing_alpha_decay 0.5

#define mel_spectogram_gain_alpha_rise 0.99
#define mel_spectogram_smoothing_alpha_rise 0.99