/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_pdm.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "i2s_pdm_example.h"
#include <stdint.h>
#include <math.h>
#include "defines.h"
#include <cstring>
#include "melSpectogram.h"
#include "ledVisualization.h"
#include <array>

#define EXAMPLE_PDM_RX_CLK_IO GPIO_NUM_0 // I2S PDM RX clock io number
#define EXAMPLE_PDM_RX_DIN_IO GPIO_NUM_4 // I2S PDM RX data in io number

#define EXAMPLE_PDM_RX_FREQ_HZ 16000 // I2S PDM RX frequency

#define MAX(a, b) ((a) > (b) ? (a) : (b))

enum pixel_tape_t
{
    tape_apa102,
    tape_ws2812,
    TAPE_MAX
};

enum lighting_mode_t
{
    random_mode,
    chase_mode
};

static const constexpr char *TAG = "dft-pdm";

static const gpio_num_t single_spi_mosi = GPIO_NUM_33;
static const gpio_num_t single_spi_sclk = GPIO_NUM_32;
static const int NUM_OUTPUTS = 1;
static const int NUM_UNIVERSES_PER_OUTPUT = 1;
static const int NUM_LEDS_PER_UNIVERSE = 60;
static const int NUM_UNIVERSES = NUM_OUTPUTS * NUM_UNIVERSES_PER_OUTPUT;
static const int speed = 2000000; // 2MHz
static const size_t APA102_BYTES_PER_LEDS = 4;
static const size_t WS2812_BYTES_PER_LEDS = 3;
static const size_t APA102_BUFFER_SIZE = APA102_BYTES_PER_LEDS * (NUM_LEDS_PER_UNIVERSE * NUM_UNIVERSES_PER_OUTPUT + 2);
static const size_t WS2812_BUFFER_SIZE = WS2812_BYTES_PER_LEDS * NUM_LEDS_PER_UNIVERSE * NUM_UNIVERSES_PER_OUTPUT;
static const size_t MAX_BUFFER_SIZE = MAX(APA102_BUFFER_SIZE, WS2812_BUFFER_SIZE);
static const int max_delay = 80; // in ms
static const int min_delay = 5;
std::array<std::array<uint8_t, 3 * NUM_LEDS_PER_UNIVERSE * NUM_UNIVERSES_PER_OUTPUT>, NUM_OUTPUTS> led_bufs;
std::array<std::array<uint8_t, MAX_BUFFER_SIZE>, NUM_OUTPUTS> output_bufs;
spi_device_handle_t single_spihand;
spi_transaction_t single_spi_tdesc;
pixel_tape_t active_tape = tape_apa102;

float audio_data_buffer[BUFFER_SIZE * N_BUFFER_ROLLING_HISTORY];
int16_t audioDataBuffer[audioBufferDataLength];
uint16_t number_of_total_samples = BUFFER_SIZE * N_BUFFER_ROLLING_HISTORY;
class MelSpectogram my_mel_spectogram(number_of_total_samples,
                                      NUMBER_OF_MEL_BIN, MIN_FREQUENCY,
                                      MAX_FREQUENCY, SAMPLE_RATE,
                                      MIN_VOLUME_THRESHOLD);

void setup_single_spi_output()
{
    spi_bus_config_t buscfg =
        {
            .mosi_io_num = single_spi_mosi,
            .miso_io_num = -1,
            .sclk_io_num = single_spi_sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = MAX_BUFFER_SIZE,
            .flags = 0,
            .intr_flags = 0,
        };
    esp_err_t ret = spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH2);
    if (ret)
    {
        ESP_LOGE(TAG, "error initializing spi bus");
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    spi_device_interface_config_t devcfg;
    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.mode = 0;
    devcfg.duty_cycle_pos = 0;
    devcfg.cs_ena_pretrans = 0;
    devcfg.cs_ena_posttrans = 0;
    devcfg.clock_speed_hz = speed;
    devcfg.clock_source = SPI_CLK_SRC_DEFAULT; // SOC_MOD_CLK_APB;
    devcfg.input_delay_ns = 0;
    devcfg.spics_io_num = -1;
    devcfg.flags = SPI_DEVICE_HALFDUPLEX;
    devcfg.queue_size = 1;
    devcfg.pre_cb = nullptr;
    devcfg.post_cb = nullptr;

    spi_bus_add_device(VSPI_HOST, &devcfg, &single_spihand);
}

void setup_spi_transaction_structs()
{
    int data_length_bits_single_spi;
    if (active_tape == tape_apa102)
    {
        data_length_bits_single_spi = (NUM_LEDS_PER_UNIVERSE * NUM_UNIVERSES_PER_OUTPUT + 2) * APA102_BYTES_PER_LEDS * 8 /* bits per byte */;
    }
    else if (active_tape == tape_ws2812)
    {
        data_length_bits_single_spi = NUM_LEDS_PER_UNIVERSE * NUM_UNIVERSES_PER_OUTPUT * WS2812_BYTES_PER_LEDS * 8 /* bits per byte */;
    }
    else
    {
        ESP_LOGE(TAG, "invalid tape type");
        assert(false);
    }

    single_spi_tdesc.flags = 0;
    single_spi_tdesc.cmd = 0;
    single_spi_tdesc.addr = 0;
    single_spi_tdesc.length = data_length_bits_single_spi;
    single_spi_tdesc.rxlength = 0;
    single_spi_tdesc.user = nullptr;
    single_spi_tdesc.tx_buffer = output_bufs[0].data();
    single_spi_tdesc.rx_buffer = nullptr;
}

void convert_to_apa102(uint8_t *apa_buffer, uint8_t *led_buffer, int num_leds)
{
    // Start frame
    memset(apa_buffer, 0, 4);

    for (int i = 0; i < num_leds; i++)
    {
        // Set global scale byte
        apa_buffer[(i + 1) * 4] = 0xff;
        // Copy rgb to bgr data
        apa_buffer[(i + 1) * 4 + 1] = led_buffer[i * 3 + 2];
        apa_buffer[(i + 1) * 4 + 2] = led_buffer[i * 3 + 1];
        apa_buffer[(i + 1) * 4 + 3] = led_buffer[i * 3 + 0];
    }
    // End frame
    memset(&apa_buffer[(num_leds + 1) * 4], 0xff, 4);
}

void update_led_bufs(uint8_t *led_buffer, int num_leds)
{
    for (int i = 0; i < num_leds; i++)
    {
        led_buffer[i * 3 + 0] = 0; // red;
        led_buffer[i * 3 + 1] = 0; // green
        led_buffer[i * 3 + 2] = 0; // blue
    }
}

void update_leds()
{
    for (int i = 0; i < NUM_OUTPUTS; i++)
    {
        // update_led_bufs(led_bufs[i].data(), NUM_LEDS_PER_UNIVERSE * NUM_UNIVERSES_PER_OUTPUT);
        convert_to_apa102(output_bufs[i].data(), led_bufs[i].data(), NUM_LEDS_PER_UNIVERSE * NUM_UNIVERSES_PER_OUTPUT);
    }
    ESP_ERROR_CHECK(spi_device_queue_trans(single_spihand, &single_spi_tdesc, portMAX_DELAY));

    spi_transaction_t *trans_desc;
    ESP_ERROR_CHECK(spi_device_get_trans_result(single_spihand, &trans_desc, portMAX_DELAY));
}

static i2s_chan_handle_t i2s_example_init_pdm_rx(void)
{
    i2s_chan_handle_t rx_chan;
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(EXAMPLE_PDM_RX_FREQ_HZ),
        /* The data bit-width of PDM mode is fixed to 16 */
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = EXAMPLE_PDM_RX_CLK_IO,
            .din = EXAMPLE_PDM_RX_DIN_IO,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_chan, &pdm_rx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    return rx_chan;
}
double mean(int16_t values[8])
{
    int sum = 0;
    for (int i = 0; i < 8; i++)
    {
        sum += values[i];
    }
    return (double)sum / 8;
}

double normalized_rms(int16_t values[8])
{
    double mean_value = mean(values);
    double sum = 0.0;
    for (int i = 0; i < 8; i++)
    {
        sum += (values[i] - mean_value) * (values[i] - mean_value);
    }
    return sqrt(sum / 8);
}

extern "C" void app_main(void)
{
    LedVisualEffects my_led_effects(NUMBER_OF_MEL_BIN, NUMBER_OF_LEDS);
    int16_t *r_buf = (int16_t *)calloc(1, EXAMPLE_BUFF_SIZE);
    assert(r_buf);
    i2s_chan_handle_t rx_chan = i2s_example_init_pdm_rx();
    size_t r_bytes = 0;
    setup_single_spi_output();
    for (auto &buf : output_bufs)
    {
        buf.fill(0);
    }

    for (auto &buf : led_bufs)
    {
        buf.fill(5);
    }
    setup_spi_transaction_structs();

    while (1)
    {
        for (int i = 0; i < N_BUFFER_ROLLING_HISTORY - 1; i++)
            memcpy(audio_data_buffer + i * BUFFER_SIZE,
                   audio_data_buffer + (i + 1) * BUFFER_SIZE,
                   sizeof(float) * BUFFER_SIZE);

        int16_t temp_audio_data_buffer[BUFFER_SIZE];
        /* Read i2s data */
        if (i2s_channel_read(rx_chan, r_buf, BUFFER_SIZE * 2, &r_bytes, 1000) == ESP_OK)
        {
            for (int i = 0; i < BUFFER_SIZE; i++)
            {
                audio_data_buffer[BUFFER_SIZE * (N_BUFFER_ROLLING_HISTORY - 1) + i] =
                    r_buf[i] / 32768.0;
            }
            static float mel_spectogram_data_buffer[NUMBER_OF_MEL_BIN];
            my_mel_spectogram.computeMelSpectogram(audio_data_buffer, mel_spectogram_data_buffer);
            my_led_effects.scroll_mel_audio_data(mel_spectogram_data_buffer, led_bufs[0].data());
            update_leds();
            // for (int i = 0; i < NUMBER_OF_MEL_BIN; i++)
            // {
            //     printf("%f ", mel_spectogram_data_buffer[i]);
            //     // Serial.print(mel_spectogram_data_buffer[i]);
            // }
            // printf("\n");
            // scroll_mel_audio_data();
        }
        else
        {
            printf("Read Task: i2s read failed\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    free(r_buf);
    vTaskDelete(NULL);
}
