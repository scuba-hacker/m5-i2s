#include <driver/i2s.h>
#include <driver/rmt.h>
#include <rom/crc.h>

#include "M5StickCPlus.h"
#include "esp_pm.h"
#include "esp_dsp.h"
#include "esp_timer.h"
#include "fft.h"

extern const unsigned char ImageData[768];
extern const unsigned char error_48[4608];

TFT_eSprite Disbuff = TFT_eSprite(&M5.Lcd);

SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t start_dis  = NULL;
SemaphoreHandle_t start_fft  = NULL;
int8_t i2s_readraw_buff[2048];
uint8_t fft_dis_buff[241][128] = {0};
uint16_t posData               = 160;

// Set to true to use the ESP-DSP FFT (Hann-windowed, assembly-optimised).
// Set to false to use the original Robin Scheibler split-radix implementation.
static const bool kUseEspDsp = false;

static float esp_dsp_fft_buf[2048];  // interleaved complex [re0,im0,re1,im1,...]
static float esp_dsp_window[1024];   // Hann window coefficients

void MicRecordfft(void *arg) {
    int16_t *buffptr;
    size_t bytesread;
    uint16_t count_n = 0;
    float adc_data;
    double data = 0;
    uint16_t ydata;

    static uint32_t frames         = 0;
    static int64_t  acc_prep_us    = 0;
    static int64_t  acc_fft_us     = 0;
    static int64_t  acc_wait_us    = 0;
    static int64_t  acc_mag_us     = 0;
    static int64_t  acc_destroy_us = 0;

    while (1) {
        xSemaphoreTake(start_fft, portMAX_DELAY);
        xSemaphoreGive(start_fft);
        fft_config_t *real_fft_plan =
            fft_init(1024, FFT_REAL, FFT_FORWARD, NULL, NULL);
        i2s_read(I2S_NUM_0, (char *)i2s_readraw_buff, 2048, &bytesread,
                 (100 / portTICK_RATE_MS));
        buffptr = (int16_t *)i2s_readraw_buff;

        int64_t t0 = esp_timer_get_time();
        for (count_n = 0; count_n < real_fft_plan->size; count_n++) {
            adc_data =
                (float)map(buffptr[count_n], INT16_MIN, INT16_MAX, -2000, 2000);
            real_fft_plan->input[count_n] = adc_data * esp_dsp_window[count_n];
        }
        int64_t t1 = esp_timer_get_time();
        fft_execute(real_fft_plan);
        int64_t t2 = esp_timer_get_time();

        xSemaphoreTake(xSemaphore, 100 / portTICK_RATE_MS);
        int64_t t3 = esp_timer_get_time();
        for (count_n = 1; count_n <= 128; count_n++) {
            float mag = sqrtf(real_fft_plan->output[2 * count_n] *
                                  real_fft_plan->output[2 * count_n] +
                              real_fft_plan->output[2 * count_n + 1] *
                                  real_fft_plan->output[2 * count_n + 1]);
            if (mag > 2000.0f) mag = 2000.0f;
            ydata = map((long)mag, 0, 2000, 0, 255);
            fft_dis_buff[posData][128 - count_n] = ydata;
        }
        posData++;
        if (posData >= 241) posData = 0;
        int64_t t4 = esp_timer_get_time();
        xSemaphoreGive(xSemaphore);
        fft_destroy(real_fft_plan);
        int64_t t5 = esp_timer_get_time();

        acc_prep_us    += t1 - t0;
        acc_fft_us     += t2 - t1;
        acc_wait_us    += t3 - t2;
        acc_mag_us     += t4 - t3;
        acc_destroy_us += t5 - t4;

        if (++frames % 100 == 0) {
            Serial.printf("[Scheibler] prep=%lldus fft=%lldus wait=%lldus mag=%lldus destroy=%lldus total=%lldus\n",
                acc_prep_us / 100, acc_fft_us / 100, acc_wait_us / 100,
                acc_mag_us / 100, acc_destroy_us / 100,
                (acc_prep_us + acc_fft_us + acc_wait_us + acc_mag_us + acc_destroy_us) / 100);
            acc_prep_us = acc_fft_us = acc_wait_us = acc_mag_us = acc_destroy_us = 0;
        }
    }
}

void MicRecordfftEspDsp(void *arg) {
    size_t bytesread;

    static uint32_t frames      = 0;
    static int64_t  acc_prep_us = 0;
    static int64_t  acc_fft_us  = 0;
    static int64_t  acc_wait_us = 0;
    static int64_t  acc_mag_us  = 0;

    while (1) {
        xSemaphoreTake(start_fft, portMAX_DELAY);
        xSemaphoreGive(start_fft);

        i2s_read(I2S_NUM_0, (char *)i2s_readraw_buff, 2048, &bytesread,
                 (100 / portTICK_RATE_MS));
        int16_t *buffptr = (int16_t *)i2s_readraw_buff;

        int64_t t0 = esp_timer_get_time();
        for (int i = 0; i < 1024; i++) {
            float s                    = (float)map(buffptr[i], INT16_MIN, INT16_MAX, -2000, 2000);
            esp_dsp_fft_buf[2 * i]     = s * esp_dsp_window[i];
            esp_dsp_fft_buf[2 * i + 1] = 0.0f;
        }
        int64_t t1 = esp_timer_get_time();
        dsps_fft2r_fc32(esp_dsp_fft_buf, 1024);
        dsps_bit_rev_fc32(esp_dsp_fft_buf, 1024);
        int64_t t2 = esp_timer_get_time();

        xSemaphoreTake(xSemaphore, 100 / portTICK_RATE_MS);
        int64_t t3 = esp_timer_get_time();
        for (int i = 1; i <= 128; i++) {
            float re   = esp_dsp_fft_buf[2 * i];
            float im   = esp_dsp_fft_buf[2 * i + 1];
            float data = sqrtf(re * re + im * im);
            if (data > 2000.0f) data = 2000.0f;
            fft_dis_buff[posData][128 - i] = (uint8_t)map((long)data, 0, 2000, 0, 255);
        }
        posData++;
        if (posData >= 241) posData = 0;
        int64_t t4 = esp_timer_get_time();
        xSemaphoreGive(xSemaphore);

        acc_prep_us += t1 - t0;
        acc_fft_us  += t2 - t1;
        acc_wait_us += t3 - t2;
        acc_mag_us  += t4 - t3;

        if (++frames % 100 == 0) {
            Serial.printf("[ESP-DSP]   prep=%lldus fft=%lldus wait=%lldus mag=%lldus total=%lldus\n",
                acc_prep_us / 100, acc_fft_us / 100, acc_wait_us / 100,
                acc_mag_us / 100,
                (acc_prep_us + acc_fft_us + acc_wait_us + acc_mag_us) / 100);
            acc_prep_us = acc_fft_us = acc_wait_us = acc_mag_us = 0;
        }
    }
}

void Drawdisplay(void *arg) {
    uint16_t count_x = 0, count_y = 0;
    uint16_t colorPos;
    while (1) {
        xSemaphoreTake(start_dis, portMAX_DELAY);
        xSemaphoreGive(start_dis);
        xSemaphoreTake(xSemaphore, 500 / portTICK_RATE_MS);
        for (count_y = 0; count_y < 128; count_y++) {
            for (count_x = 0; count_x < 240; count_x++) {
                if ((count_x + (posData % 240)) > 240) {
                    colorPos =
                        fft_dis_buff[count_x + (posData % 240) - 240][count_y];
                } else {
                    colorPos = fft_dis_buff[count_x + (posData % 240)][count_y];
                }

                Disbuff.drawPixel(
                    count_x, count_y,
                    Disbuff.color565(ImageData[colorPos * 3 + 0],
                                     ImageData[colorPos * 3 + 1],
                                     ImageData[colorPos * 3 + 2]));
                /*
                disbuff[ count_y * 160 + count_x ].r =  ImageData[ colorPos * 3
                + 0 ]; disbuff[ count_y * 160 + count_x ].g =  ImageData[
                colorPos * 3 + 1 ]; disbuff[ count_y * 160 + count_x ].b =
                ImageData[ colorPos * 3 + 2 ];
                */
            }
        }
        xSemaphoreGive(xSemaphore);
        Disbuff.pushSprite(0, 0);
    }
}

TaskHandle_t xhandle_display = NULL;
TaskHandle_t xhandle_fft     = NULL;

void DisplayMicro() {
    Disbuff.fillRect(0, 0, 160, 80, Disbuff.color565(0, 0, 0));
    Disbuff.pushSprite(0, 0);

    xSemaphoreGive(start_dis);
    xSemaphoreGive(start_fft);
    while ((!M5.BtnA.isPressed()) && (!M5.BtnB.isPressed())) {
        xSemaphoreGive(start_dis);
        xSemaphoreGive(start_fft);
        M5.update();
        // delay(100);
        xSemaphoreTake(start_dis, portMAX_DELAY);
        xSemaphoreTake(start_fft, portMAX_DELAY);
    }
    // xSemaphoreTake( start_dis , portMAX_DELAY  );
    // xSemaphoreTake( start_fft , portMAX_DELAY  );

    delay(50);
}

#define PIN_CLK  0
#define PIN_DATA 34

bool InitI2SMicroPhone() {
    esp_err_t err           = ESP_OK;
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = 44100,
        .bits_per_sample =
            I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
        .communication_format =
            I2S_COMM_FORMAT_STAND_I2S,  // Set the format of the communication.
#else                                   // 设置通讯格式
        .communication_format = I2S_COMM_FORMAT_I2S,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count    = 2,
        .dma_buf_len      = 128,
    };

    i2s_pin_config_t pin_config;
#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
    pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif
    pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
    pin_config.ws_io_num    = PIN_CLK;
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;
    pin_config.data_in_num  = PIN_DATA;

    err += i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    err += i2s_set_pin(I2S_NUM_0, &pin_config);
    err += i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT,
                       I2S_CHANNEL_MONO);
    // i2s_set_clk(0)

    if (err != ESP_OK) {
        return false;
    } else {
        return true;
    }
}

void setup() {
    M5.begin();
    M5.Lcd.setRotation(3);
    Disbuff.createSprite(240, 135);
    M5.update();
    M5.Axp.ScreenBreath(100);
    M5.Imu.Init();
    InitI2SMicroPhone();
    dsps_fft2r_init_fc32(NULL, 1024);
    dsps_wind_hann_f32(esp_dsp_window, 1024);

    xSemaphore = xSemaphoreCreateMutex();
    start_dis  = xSemaphoreCreateMutex();
    start_fft  = xSemaphoreCreateMutex();

    xSemaphoreTake(start_dis, portMAX_DELAY);
    xSemaphoreTake(start_fft, portMAX_DELAY);

    xTaskCreate(Drawdisplay, "Drawdisplay", 1024 * 2, (void *)0, 4,
                &xhandle_display);
    xTaskCreate(kUseEspDsp ? MicRecordfftEspDsp : MicRecordfft,
                "MicRecordfft", 1024 * 2, (void *)0, 5, &xhandle_fft);

    Disbuff.pushSprite(0, 0);
}

void loop() {
    DisplayMicro();
    M5.update();
    delay(50);
}
