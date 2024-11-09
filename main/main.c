/**
 * \file main.c
 * \mainpage Example of reading out the camera on the Seeed Studio Xiao ESP32-S3 Sense
 * \author Tobit Flatscher (github.com/2b-t)
*/

#include <sys/param.h>

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#include "esp_camera.h"

// Change the camera model here
#define CAMERA_MODEL_XIAO_ESP32S3 1
#include "boards.h"


static const char* tag = "main";

static const int frame_delay = 1000;

static const camera_config_t camera_config = {
  .pin_pwdn = CAMERA_PIN_PWDN,
  .pin_reset = CAMERA_PIN_RESET,
  .pin_xclk = CAMERA_PIN_XCLK,
  .pin_sccb_sda = CAMERA_PIN_SIOD,
  .pin_sccb_scl = CAMERA_PIN_SIOC,

  .pin_d7 = CAMERA_PIN_D7,
  .pin_d6 = CAMERA_PIN_D6,
  .pin_d5 = CAMERA_PIN_D5,
  .pin_d4 = CAMERA_PIN_D4,
  .pin_d3 = CAMERA_PIN_D3,
  .pin_d2 = CAMERA_PIN_D2,
  .pin_d1 = CAMERA_PIN_D1,
  .pin_d0 = CAMERA_PIN_D0,
  .pin_vsync = CAMERA_PIN_VSYNC,
  .pin_href = CAMERA_PIN_HREF,
  .pin_pclk = CAMERA_PIN_PCLK,

  .xclk_freq_hz = 10000000, // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_RGB565, // YUV422, GRAYSCALE, RGB565 or JPEG (not supported on OV2640)
  .frame_size = FRAMESIZE_QQVGA, // QQVGA, UXGA

  .jpeg_quality = 12, // 0-63, lower number means higher quality
  .fb_count = 1, // In JPEG mode if fb_count is more than one, the driver will work in continuous mode
  .fb_location = CAMERA_FB_IN_PSRAM, // CAMERA_FB_IN_PSRAM or CAMERA_FB_IN_DRAM
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};


void app_main(void) {
  if(esp_camera_init(&camera_config) != ESP_OK) {
    ESP_LOGE(tag, "Camera Init Failed");
    return;
  }

  while(true) {
    ESP_LOGI(tag, "Taking picture...");
    camera_fb_t* pic = esp_camera_fb_get();

    // use pic->buf to access the image
    ESP_LOGI(tag, "Picture taken! Its size was: %zu bytes", pic->len);
    esp_camera_fb_return(pic);

    vTaskDelay(frame_delay / portTICK_RATE_MS);
 }
}

