/**
 * \file main.c
 * \mainpage Example of reading out the camera on the Seeed Studio Xiao ESP32-S3 Sense
 * \author Tobit Flatscher (github.com/2b-t)
*/

#include <sys/param.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>

#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
  #include <rmw_microros/rmw_microros.h>
#endif
#include <sensor_msgs/msg/image.h>
#include <uros_network_interfaces.h>

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#include "esp_camera.h"

// Change the camera model here
#define CAMERA_MODEL_XIAO_ESP32S3 1
#include "boards.h"


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

  .pixel_format = PIXFORMAT_GRAYSCALE, // YUV422, GRAYSCALE, RGB565 or JPEG (not supported on OV2640)
  .frame_size = FRAMESIZE_QQVGA, // QQVGA, UXGA

  .jpeg_quality = 12, // 0-63, lower number means higher quality
  .fb_count = 1, // In JPEG mode if fb_count is more than one, the driver will work in continuous mode
  .fb_location = CAMERA_FB_IN_PSRAM, // CAMERA_FB_IN_PSRAM or CAMERA_FB_IN_DRAM
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
sensor_msgs__msg__Image msg;


void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    printf("Taking picture...\n");
    camera_fb_t* pic = esp_camera_fb_get();
    printf("Picture taken! Its size was: %zu bytes\n", pic->len);

    //if (pic->len <= msg.data.capacity) {
    if (true) {
      msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, "camera");
      // See https://github.com/espressif/esp32-camera/blob/master/driver/include/sensor.h for resolutions
      msg.width = 160;
      msg.height = 120;
      msg.step = msg.width;
      // See https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/include/sensor_msgs/image_encodings.hpp
      msg.encoding = micro_ros_string_utilities_set(msg.encoding, "mono8");
      msg.data.size = pic->len;
      memcpy(msg.data.data, pic->buf, pic->len);

      printf("Publishing image!\n");
      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    } else {
      printf("Failed to publish image (image size %zu, max. capacity %zu)!\n", pic->len, msg.data.capacity);
    }

    esp_camera_fb_return(pic);
	}
  return;
}

void micro_ros_task(void* arg){
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

  RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "esp32_image_publisher", "", &support));

  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image), "image_publisher"));

  static micro_ros_utilities_memory_conf_t conf = {};
  conf.max_string_capacity = 50;
  conf.max_ros2_type_sequence_capacity = 5;
  conf.max_basic_type_sequence_capacity = 5;
  micro_ros_utilities_memory_rule_t const rules[] = {
    {"header.frame_id", 30},
    {"format", 5},
    {"data", 19200}
  };
  conf.rules = rules;
  conf.n_rules = sizeof(rules) / sizeof(rules[0]);

  micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
    &msg,
    conf
  );

  rcl_timer_t timer;
  unsigned int const timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  while(true){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    usleep(10000);
  }

  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_node_fini(&node));

  vTaskDelete(NULL);
  return;
}

void app_main(void) {
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
  ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

  if (esp_camera_init(&camera_config) != ESP_OK) {
    printf("Failed to initialize camera!\n");
    esp_restart();
    return;
  }

  xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}

