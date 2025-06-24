#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l5cx_api.h"

#include "esp_log.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"

 #include "esp_task_wdt.h"
 #define WATCHDOG_TIMEOUT_SEC 5  // Reset if no feed within 10 seconds

 
////// LED
 #include "led_strip.h"
 #include "led_strip_rmt.h"
 #include "math.h"

 #include "driver/gpio.h"

 // gyro
 #include <mpu9250.h>
#include <math.h> // For atan2f and sqrtf

// --- Alert Level Types ---
typedef enum {
    ALERT_SILENT = 0,
    ALERT_VERYFAR,
    ALERT_FAR,
    ALERT_MEDIUMFAR,
    ALERT_MEDIUM,
    ALERT_CLOSE,
    ALERT_IMMEDIATE
} alert_level_t; // Now this is defined

enum class LedColor {
    OFF, RED, ORANGE, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, BLACK
};

// --- Data Structures ---
typedef struct {
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float pitch, roll, temp_c;
} mpu_data_t;

typedef struct {
    uint32_t frequency_hz, 
    beep_duration_ms, 
    beep_interval_ms;
    const char* description;
    LedColor led_color;
    int led_blink_times, 
    led_blink_delay_ms;
    float led_strength_val;
} alert_config_t;

typedef struct {
    alert_level_t current_level;
    bool is_beeping;
    uint32_t beep_start_time, last_beep_time, beep_count;
} beeper_state_t;


// ----------------------------------------------------------------
// --- NEW: Single LED State Struct and Mode Enum ---
// ----------------------------------------------------------------

// Defines what the WS2812 LED part of the state should be doing
typedef enum {
    LED_MODE_OFF,
    LED_MODE_SOLID,
    LED_MODE_BLINK,
} led_mode_t;

/*
// This single struct describes the complete desired state for ALL LEDs
typedef struct {
    led_mode_t ws2812_mode; // The mode for the main RGB LED
    uint8_t r, g, b;
    float brightness;
    uint32_t blink_period_ms;

    bool top_led_on;      // The state for the Red GPIO LED
    bool bottom_led_on;   // The state for the Blue GPIO LED

} led_state_t;
*/


// This single struct describes the complete desired state for ALL LEDs.
// We have removed the complex "mode" enum.
// ----------------------------------------------------------------
// --- ACTION: Replace your LED State Struct with this version ---
// ----------------------------------------------------------------
struct led_state_t { // Give the struct its name at the beginning
    uint8_t r = 0, g = 0, b = 0;
    float brightness = 0.0f;
    bool top_led_on = false;
    bool bottom_led_on = false;

    // Now, inside the struct, the name 'led_state_t' is already known.
    // This will compile correctly.
    bool operator!=(const led_state_t& other) const {
        return r != other.r || 
               g != other.g || 
               b != other.b || 
               brightness != other.brightness ||
               top_led_on != other.top_led_on ||
               bottom_led_on != other.bottom_led_on;
    }
};


// The handle for our queue, which will now carry this new struct
QueueHandle_t led_state_queue;

typedef struct {
    VL53L5CX_Configuration *dev;
    const uint8_t (*gridLayout)[4];
    alert_level_t *gridAlerts;
} sensor_task_params_t;

static const alert_config_t alert_configs[] = {
    {0,    0,   0,    "Silent",    LedColor::BLACK,  1, 100, 0}, // ALERT_SILENT
    {500,  20,  100, "VeryFar",         LedColor::BLUE,   1, 200, 0}, // ALERT_VERYFAR
    {800,  50,  50, "Far",         LedColor::GREEN,   1, 200, 0.6}, // ALERT_FAR
    {1000,  50,  50, "MediumFar",  LedColor::YELLOW,   3, 200, 0.6}, // ALERT_MEDIUMFAR
    {1200, 50,  80, "Medium",      LedColor::ORANGE, 3, 300, 0.7}, // ALERT_MEDIUM
    {1800, 50,  120,  "Close",     LedColor::RED,    5, 200, 0.8}, // ALERT_CLOSE
    {2500, 70, 200,  "Immediate",  LedColor::WHITE,  5, 100, 0.8}  // ALERT_IMMEDIATE
};

#define RED_PIN     GPIO_NUM_21
#define BLUE_PIN  GPIO_NUM_47
#define GREEN_PIN  GPIO_NUM_45

#define RED_PIN_1     GPIO_NUM_35
#define BLUE_PIN_1  GPIO_NUM_36
#define GREEN_PIN_1  GPIO_NUM_37


 #define WS2812_GPIO         48
 #define WS2812_LED_COUNT     1
 
static const char *TAG = "WALKING_AID";

static led_strip_handle_t led_strip = NULL;
// Piezo beeper pin
#define PIEZO_BEEPER_PIN        GPIO_NUM_38   // PWM pin for piezo beeper

// Detection parameters
#define MOVING_AVERAGE_WINDOW_SIZE  15

// PWM configuration for piezo beeper
#define BEEPER_LEDC_TIMER       LEDC_TIMER_0
#define BEEPER_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define BEEPER_LEDC_CHANNEL     LEDC_CHANNEL_0
#define BEEPER_DUTY_RESOLUTION  LEDC_TIMER_10_BIT  // 0-1023 duty range

// Audio alert parameters
#define BEEP_DUTY_CYCLE         512  // 50% duty cycle for clear tone

//static bool useAcc = 0;
//#define  useAccelerometer 0
#undef useAccelerometer
#undef useRGBLed


 int ALERT_IMMEDIATE_LIMIT = 40;
  int ALERT_CLOSE_LIMIT  = 80;
  int ALERT_MEDIUM_LIMIT  = 100;
   int ALERT_MEDIUMFAR_LIMIT  = 140;
  int ALERT_FAR_LIMIT  = 200;
    int ALERT_VERYFAR_LIMIT  = 300;
#define HYSTERESIS 15  // cm

   // Static arrays for grid layout and alerts
    static const uint8_t gridLayout[4][4] = {
        {0, 1, 2, 3},
        {4, 5, 6, 7},
        {8, 9, 10, 11},
        {12, 13, 14, 15}
    };
    static alert_level_t gridAlerts[16];


#ifdef useRGBLed
SemaphoreHandle_t led_strip_mutex;
#endif
SemaphoreHandle_t g_i2c_bus_mutex;

// ----------------------------------------------------------------
// --- NEW: Function Prototypes / Forward Declarations ---
// ----------------------------------------------------------------

// Low-level LED helper functions
#ifdef useRGBLed
void rgb_led_init();
void rgb_led_off(); // <-- This is the missing declaration
void rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b);
void rgb_led_set_color_with_brightness(uint8_t r, uint8_t g, uint8_t b, float brightness);
uint8_t gamma_correct(uint8_t val);
#endif

// Beeper helper functions
esp_err_t init_piezo_beeper(void);
void set_beeper_tone(uint32_t frequency_hz, bool enable);

// Alert logic functions
alert_level_t get_alert_level(uint16_t distance_cm);
void update_beeper_alerts(alert_level_t new_level);

// Task functions

#ifdef useRGBLed
void led_control_task(void *pvParameters);
#endif
#ifdef useAccelerometer
void mpu9250_reader_task(void *pvParameters);
void data_processor_task(void *pvParameters);
#endif

void vl53l5cx_reader_task(void *pvParameters);
bool vl53l5cx_recover(VL53L5CX_Configuration *dev);

#ifdef useRGBLed
void testblink(){
  led_strip_set_pixel(led_strip, 0, 16, 0, 0);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);

          vTaskDelay(pdMS_TO_TICKS(300));
  led_strip_set_pixel(led_strip, 0, 0, 16, 0);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
 vTaskDelay(pdMS_TO_TICKS(300));
          led_strip_set_pixel(led_strip, 0, 0, 0, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
  vTaskDelay(pdMS_TO_TICKS(300));

 led_strip_clear(led_strip);
}
#endif

/*
void watchdog_feed() {
    wdt_feed();
}
//esp_task_wdt_feed()

void init_watchdog() {
    wdt_enable( WDT_PERIPH_TIMEOUT_S( 60 )); // 60 seconds timeout
    xTaskCreate(&watchdog_feed_task, "wdt_feed", 2048, NULL, 1, NULL);
}

void watchdog_feed_task(void *pvParameters) {
    while(1) {
        watchdog_feed();
        vTaskDelay( pdMS_TO_TICKS( 50000 )); // Feed every 50 seconds
    }
}
*/

#ifdef useRGBLed
/**
 * @brief Helper function to print the contents of an led_state_t struct
 * 
 * @param tag The ESP_LOG tag to use.
 * @param prefix A string to print before the struct data.
 * @param state The struct to print.
 */
void log_led_state(const char* tag, const char* prefix, const led_state_t& state)
{
    ESP_LOGI(tag, "%s r=%u, g=%u, b=%u, bright=%.2f, top=%s, bot=%s",
             prefix,
             state.r,
             state.g,
             state.b,
             state.brightness,
             (state.top_led_on ? "ON" : "OFF"),
             (state.bottom_led_on ? "ON" : "OFF"));
}
// ----------------------------------------------------------------
// --- NEW: LED Control Task Definitions ---
// ----------------------------------------------------------------

// The handle for our command queue
//QueueHandle_t led_command_queue;

// ----------------------------------------------------------------
// --- FINAL, CORRECTED led_control_task (The "Renderer") ---
// ----------------------------------------------------------------
// -------------------------------------------------------------------------
// --- DEFINITIVE, CORRECTED led_control_task ---
// -------------------------------------------------------------------------
// ----------------------------------------------------------------
// --- FINAL, Rock-Solid led_control_task (The "Renderer") ---
// ----------------------------------------------------------------
// ----------------------------------------------------------------
// --- DEFINITIVE, Rock-Solid, Event-Driven led_control_task ---
// ----------------------------------------------------------------
void led_control_task(void *pvParameters)
{
    // A variable to hold the new state when it arrives from the queue.
    led_state_t new_state{}; 

    ESP_LOGI("LED_TASK", "Event-Driven Renderer Task Started. Waiting for state updates...");

    // This is the entire task.
    while (1)
    {
        // Block and wait INDEFINITELY until a new state is received from the queue.
        // This is extremely efficient; the task uses 0% CPU while waiting.
        if (xQueueReceive(led_state_queue, &new_state, portMAX_DELAY) == pdPASS) 
        {
            // --- A new state has arrived. Render it ONCE. ---
            
            ESP_LOGI("LED_TASK", "New state received. Updating hardware.");
            log_led_state("LED_TASK", "New State:", new_state); // Using the helper function to log

            // Render GPIO LEDs based on the new state
            gpio_set_level(RED_PIN, new_state.top_led_on);
            gpio_set_level(BLUE_PIN, new_state.bottom_led_on);

            // Render WS2812 LED based on the new state
            if (new_state.brightness > 0.0f) {
                ESP_LOGI("LED_TASK", "brightness > 0");
                rgb_led_set_color_with_brightness(new_state.r, new_state.g, new_state.b, new_state.brightness);
            } else {
                ESP_LOGI("LED_TASK", "brightness < 0");
                rgb_led_off();
            }

            // After rendering, the task loops back and immediately sleeps again,
            // waiting for the next message. There is no vTaskDelay here.
        }
    }
}

void led_control_task_old(void *pvParameters)
{
    // This variable holds the last known "desired state" received from the queue.
    led_state_t current_state{}; // Starts with all LEDs off

    ESP_LOGI("LED_TASK", "Renderer Task Started.");

    while (1)
    {
        // Check if a new desired state has been sent. This is non-blocking.
        xQueueReceive(led_state_queue, &current_state, 0);

        // --- Render the GPIO LEDs state ---
        gpio_set_level(RED_PIN, current_state.top_led_on);
        gpio_set_level(BLUE_PIN, current_state.bottom_led_on);

        // --- Render the WS2812 LED state ---
        // If brightness is 0, we can turn it off. Otherwise, set the color.
        if (current_state.brightness > 0.0f) {

            ESP_LOGI("LED_TASK", "current_state.brightness > 0");
            rgb_led_set_color_with_brightness(current_state.r, current_state.g, current_state.b, current_state.brightness);
        } else {
            rgb_led_off();
        }
        
        // Loop at a reasonable "refresh rate" for the LEDs.
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
#endif

#ifdef useAccelerometer
#define MPU9250_I2C_ADDRESS 0x68


// --- Filter state variables ---
// These MUST be static or global so they retain their value across task iterations.
static float current_pitch = 0.0f;
static float current_roll = 0.0f;

// A queue to safely pass the sensor data from the reader task to other tasks
QueueHandle_t mpu_data_queue;
#define MPU9250_PWR_MGMT_1_REG_ADDR   0x6B
#define MPU9250_ACCEL_XOUT_H_REG_ADDR 0x3B // Start of all sensor data

// --- MPU9250 Sensitivity Scale Factors (for default settings) ---
// Default Accel Range: ±2g -> 16384.0 LSB/g
#define ACCEL_SCALE_FACTOR 16384.0f
// Default Gyro Range: ±250 °/s -> 131.0 LSB/(°/s)
#define GYRO_SCALE_FACTOR  131.0f

// Complementary filter coefficient (ALPHA). A higher value means you trust the gyro more.
// 0.98 is a common starting point for responsive yet stable tracking.
#define COMPLEMENTARY_FILTER_ALPHA 0.98f

#endif

#ifdef useAccelerometer
/**
 * @brief FreeRTOS task to read MPU9250 sensor data and calculate a stable
 *        orientation using a complementary filter.
 * 
 * @param pvParameters The device handle for the MPU9250 I2C connection.
 */
void mpu9250_reader_task(void *pvParameters)
{
    i2c_master_dev_handle_t mpu9250_dev_handle = (i2c_master_dev_handle_t)pvParameters;
    uint8_t raw_data[14]; // Buffer for all accel, temp, and gyro data
    mpu_data_t sensor_data;

    uint32_t last_update_time = xTaskGetTickCount();

    ESP_LOGI("MPU9250_TASK", "Fusion task started.");

    while (1)
    {
        uint32_t current_time = xTaskGetTickCount();
        float dt = (float)(current_time - last_update_time) / (float)configTICK_RATE_HZ;
        last_update_time = current_time;
 
        
           if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY) == pdTRUE) {
            


        esp_err_t err = i2c_master_transmit_receive(
            mpu9250_dev_handle,
            (const uint8_t[]){MPU9250_ACCEL_XOUT_H_REG_ADDR},
            1,
            raw_data,
            14,
            pdMS_TO_TICKS(100)
        );

        if (err == ESP_OK)
        {
            // --- Parse raw data and convert to physical units ---
            int16_t raw_accel_x = (raw_data[0] << 8) | raw_data[1];
            int16_t raw_accel_y = (raw_data[2] << 8) | raw_data[3];
            int16_t raw_accel_z = (raw_data[4] << 8) | raw_data[5];
            int16_t raw_temp    = (raw_data[6] << 8) | raw_data[7]; // Temperature data
            int16_t raw_gyro_x  = (raw_data[8] << 8) | raw_data[9];
            int16_t raw_gyro_y  = (raw_data[10] << 8) | raw_data[11];
            int16_t raw_gyro_z  = (raw_data[12] << 8) | raw_data[13];

            // Convert raw values
            sensor_data.accel_x_g = (float)raw_accel_x / ACCEL_SCALE_FACTOR;
            sensor_data.accel_y_g = (float)raw_accel_y / ACCEL_SCALE_FACTOR;
            sensor_data.accel_z_g = (float)raw_accel_z / ACCEL_SCALE_FACTOR;
            sensor_data.gyro_x_dps  = (float)raw_gyro_x / GYRO_SCALE_FACTOR;
            sensor_data.gyro_y_dps  = (float)raw_gyro_y / GYRO_SCALE_FACTOR;
            sensor_data.gyro_z_dps  = (float)raw_gyro_z / GYRO_SCALE_FACTOR;
            
            // ***************************************************************
            // *** ADDED THIS LINE BACK IN ***
            // Temperature formula from the MPU9250 register map datasheet
            sensor_data.temp_c  = ((float)raw_temp / 333.87f) + 21.0f;
            // ***************************************************************

            // --- Calculate Pitch and Roll from Accelerometer ---
            float pitch_from_accel = atan2f(sensor_data.accel_y_g, sensor_data.accel_z_g) * (180.0f / M_PI);
            float roll_from_accel = atan2f(-sensor_data.accel_x_g, sqrtf(sensor_data.accel_y_g * sensor_data.accel_y_g + sensor_data.accel_z_g * sensor_data.accel_z_g)) * (180.0f / M_PI);

            // --- THE COMPLEMENTARY FILTER ---
            current_pitch = COMPLEMENTARY_FILTER_ALPHA * (current_pitch + sensor_data.gyro_x_dps * dt) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * pitch_from_accel;
            current_roll  = COMPLEMENTARY_FILTER_ALPHA * (current_roll + sensor_data.gyro_y_dps * dt) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * roll_from_accel;

            // --- Populate the struct and send it to the queue ---
            sensor_data.pitch = current_pitch;
            sensor_data.roll = current_roll;
            
            xQueueSend(mpu_data_queue, &sensor_data, 0);

        } else {
            ESP_LOGE("MPU9250_TASK", "Failed to read sensor data: %s", esp_err_to_name(err));
        }

//////  fall detection
    float total_accel_magnitude = sqrtf(
        sensor_data.accel_x_g * sensor_data.accel_x_g +
        sensor_data.accel_y_g * sensor_data.accel_y_g +
        sensor_data.accel_z_g * sensor_data.accel_z_g
    );

// When the device is still, magnitude should be ~1.0. During free-fall, it's ~0.0.
    if (total_accel_magnitude < 0.2) { // Threshold for detecting free-fall
        ESP_LOGW("FALL_DETECT", "FREE-FALL DETECTED! Magnitude: %.2f g", total_accel_magnitude);
    // You could trigger an alert here or set a flag for another task to see.
    }

                  // ALWAYS give the mutex back
            xSemaphoreGive(g_i2c_bus_mutex);


 vTaskDelay(pdMS_TO_TICKS(300));
      
    }

  
}
    
     
}
#endif

#ifdef useAccelerometer
/**
 * @brief Example task that waits for and processes MPU9250 data from the queue.
 *        This task demonstrates how to correctly access the fused orientation data.
 */

void data_processor_task(void *pvParameters)
{
    mpu_data_t received_data;
    ESP_LOGI("PROCESSOR_TASK", "Task started, waiting for data.");

    while (1)
    {
        // Wait indefinitely until an item is available on the queue.
        if (xQueueReceive(mpu_data_queue, &received_data, portMAX_DELAY) == pdPASS)
        {
            // --- FUSED ORIENTATION DATA (Most Important Output) ---
            ESP_LOGI("PROCESSOR_TASK", "Orientation -> Pitch: %.2f°, Roll: %.2f°", received_data.pitch, received_data.roll);
        }
 vTaskDelay(pdMS_TO_TICKS(300));


    }
}
#endif

#ifdef useRGBLed
void rgb_led_init() {
    ESP_LOGI("LED_INIT", "Initializing WS2812 LED strip...");
    
    // RMT (Remote Control) peripheral configuration for the LED strip
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz resolution, good for WS2812
        .flags = {
            .with_dma = false, // DMA is not needed for a single LED
        }
    };

    // LED strip configuration
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,           // The GPIO the LED is connected to
        .max_leds = 1,                           // The number of LEDs in the strip
        .led_model = LED_MODEL_WS2812,           // Set the LED model to WS2812
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // Set the color order
    };

    // Create the LED strip handle
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
 
    if (err != ESP_OK || led_strip == NULL) {
        ESP_LOGE("LED_INIT", "LED strip init failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("LED_INIT", "LED strip initialized successfully.");
        // Clear the strip to ensure it's off at boot.
        led_strip_clear(led_strip);
    }
}



static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
               .flags = {
            .with_dma = false, // DMA is not needed for a single LED
        }

//        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
 led_strip_clear(led_strip);
}

const char* led_color_to_string(LedColor color) {
    switch (color) {
        case LedColor::BLACK:    return "BLACK";
        case LedColor::GREEN:    return "GREEN";
        case LedColor::YELLOW:   return "YELLOW";
        case LedColor::ORANGE:  return "ORANGE";
        case LedColor::RED:     return "RED";
        case LedColor::WHITE:   return "WHITE"; // Corrected position
        case LedColor::BLUE:    return "BLUE";
        case LedColor::CYAN:    return "CYAN"; // Added
        case LedColor::MAGENTA: return "MAGENTA"; // Added
        default:                return "UNKNOWN_COLOR";
    }
}

void rgb_led_off() {
    // Take the mutex. Wait up to 10ms to get it.
    if (xSemaphoreTake(led_strip_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        led_strip_clear(led_strip);

        // ALWAYS give the mutex back.
        xSemaphoreGive(led_strip_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to get LED mutex in off!");
    }
}
 
 void rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b) {
     esp_err_t  err ;
  ESP_LOGI("LED", "rgb_led_set_color %d %d %d", r, g, b);
     if (led_strip == nullptr) {
         ESP_LOGE("LED", "led_strip is NULL, skipping rgb_led_set_color");
         return;
     }

  led_strip_clear(led_strip);

       err =  led_strip_set_pixel(led_strip, 0, r, g, b);
       if (err != ESP_OK || led_strip == NULL) {
         ESP_LOGE("LED", "led_strip_set_pixel failed: %s", esp_err_to_name(err));
     } else {
        // ESP_LOGI("LED", "led_strip_set_pixel successful: %s", esp_err_to_name(err));
  //   ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
     }
     
  
       err =  led_strip_refresh(led_strip);
       if (err != ESP_OK || led_strip == NULL) {
         ESP_LOGE("LED", "led_strip_refresh failed: %s", esp_err_to_name(err));
     } else {
       //  ESP_LOGI("LED", "led_strip_refresh successful: %s", esp_err_to_name(err));
  //   ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
     }
 }
 
 
 uint8_t gamma_correct(uint8_t val) {
     return (uint8_t)(std::powf(val / 255.0f, 2.2f) * 255.0f);
 }
 
 void rgb_led_set_color_with_brightness(uint8_t r, uint8_t g, uint8_t b, float brightness) {

  if (xSemaphoreTake(led_strip_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

     if (brightness < 0.0f) brightness = 0.0f;
     if (brightness > 1.0f) brightness = 1.0f;
 
     uint8_t scaled_r = gamma_correct((uint8_t)(r * brightness));
     uint8_t scaled_g = gamma_correct((uint8_t)(g * brightness));
     uint8_t scaled_b = gamma_correct((uint8_t)(b * brightness));
 
        ESP_LOGI(TAG, "rgb_led_set_color_with_brightness: %d %d %d %f", scaled_r, scaled_g, scaled_b, brightness);

     //  led_strip_clear(led_strip);
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, scaled_r, scaled_g, scaled_b));
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));

         // ALWAYS give the mutex back.
        xSemaphoreGive(led_strip_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to get LED mutex in set_color!");
    }

 }
 
 void rgb_led_blink(LedColor color, int times, int ms_delay, float strength_val) {
     uint8_t r = 0, g = 0, b = 0;
 
     switch (color) {
         case LedColor::RED:      r = 255; g = 0;   b = 0;   break;
         case LedColor::ORANGE:   r = 255; g = 140;   b = 0;   break; //255,140,0
         case LedColor::GREEN:    r = 0;   g = 255; b = 0;   break; 
         case LedColor::BLUE:     r = 0;   g = 0;   b = 255; break;
         case LedColor::YELLOW:   r = 255; g = 255; b = 0;   break;
         case LedColor::CYAN:     r = 0;   g = 255; b = 255; break;
         case LedColor::MAGENTA:  r = 180; g = 100;   b = 255; break;
         case LedColor::WHITE:    r = 255; g = 255; b = 255; break; 
         case LedColor::BLACK:    r = 0;   g = 0;   b = 0;   break;
         default:                 r = 0;   g = 0;   b = 0;   break;
     }
 
     for (int i = 0; i < times; ++i) {
        ESP_LOGI("rgb_led_blink", "repeat %d of %d", i, times );
         rgb_led_set_color_with_brightness(r, g, b, strength_val);
         vTaskDelay(pdMS_TO_TICKS(ms_delay));
      //   rgb_led_off();
         vTaskDelay(pdMS_TO_TICKS(ms_delay));
     }
 }
 
void rgb_led_constant(LedColor color, float strength_val) {
     uint8_t r = 0, g = 0, b = 0;
 
     switch (color) {
         case LedColor::RED:      r = 255; g = 0;   b = 0;   break;
         case LedColor::ORANGE:   r = 255; g = 140;   b = 0;   break; //255,140,0
         case LedColor::GREEN:    r = 0;   g = 255; b = 0;   break; 
         case LedColor::BLUE:     r = 0;   g = 0;   b = 255; break;
         case LedColor::YELLOW:   r = 255; g = 255; b = 0;   break;
         case LedColor::CYAN:     r = 0;   g = 255; b = 255; break;
         case LedColor::MAGENTA:  r = 180; g = 100;   b = 255; break;
         case LedColor::WHITE:    r = 255; g = 255; b = 255; break; 
         case LedColor::BLACK:    r = 0;   g = 0;   b = 0;   break;
         default:                 r = 0;   g = 0;   b = 0;   break;
     }
 
         ESP_LOGI(TAG, "rgb_led_constant: Color:%d %d %d", r, g, b);

  //   for (int i = 0; i < times; ++i) {
       vTaskDelay(pdMS_TO_TICKS(10));
         rgb_led_set_color_with_brightness(r, g, b, strength_val);
       vTaskDelay(pdMS_TO_TICKS(5));
   //      rgb_led_off();
   //      vTaskDelay(pdMS_TO_TICKS(ms_delay));
 //    }
 }
 
#endif



static beeper_state_t beeper = {};

esp_err_t init_piezo_beeper(void) {
    // Configure LEDC timer
    ledc_timer_config_t timer_config = {
        .speed_mode = BEEPER_LEDC_MODE,  
        .duty_resolution = BEEPER_DUTY_RESOLUTION,
        .timer_num = BEEPER_LEDC_TIMER,
         .freq_hz = 1000,  // Default frequency, will be changed dynamically
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    // Configure LEDC channel
    ledc_channel_config_t channel_config = {
         .gpio_num = PIEZO_BEEPER_PIN,
             .speed_mode = BEEPER_LEDC_MODE,
        .channel = BEEPER_LEDC_CHANNEL,
   //     .intr_type
        .timer_sel = BEEPER_LEDC_TIMER,
        .duty = 0,  // Start silent
        .hpoint = 0,
        
    };

   ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    ESP_LOGI(TAG, "Piezo beeper initialized on GPIO%d", PIEZO_BEEPER_PIN);
    return ESP_OK;
}

// Set beeper frequency and enable/disable
void set_beeper_tone(uint32_t frequency_hz, bool enable) {
    if (enable && frequency_hz > 0) {
        // Set frequency
        ledc_set_freq(BEEPER_LEDC_MODE, BEEPER_LEDC_TIMER, frequency_hz);
        // Set duty cycle for audible tone
        ledc_set_duty(BEEPER_LEDC_MODE, BEEPER_LEDC_CHANNEL, BEEP_DUTY_CYCLE);
        ledc_update_duty(BEEPER_LEDC_MODE, BEEPER_LEDC_CHANNEL);
    } else {
        // Silence the beeper
        ledc_set_duty(BEEPER_LEDC_MODE, BEEPER_LEDC_CHANNEL, 0);
        ledc_update_duty(BEEPER_LEDC_MODE, BEEPER_LEDC_CHANNEL);
    }
}

// Determine alert level based on distance
alert_level_t get_alert_level(uint16_t distance_cm) {
    if (distance_cm <= ALERT_IMMEDIATE_LIMIT ) { //+ HYSTERESIS
        return ALERT_IMMEDIATE;
    } else if (distance_cm <= ALERT_CLOSE_LIMIT ) {
        return ALERT_CLOSE;
    } else if (distance_cm <= ALERT_MEDIUM_LIMIT ) {
        return ALERT_MEDIUM;
      } else if(distance_cm <= ALERT_MEDIUMFAR_LIMIT){
        return(ALERT_MEDIUMFAR);
    } else if (distance_cm <= ALERT_FAR_LIMIT ) {
        return ALERT_FAR;
   } else if (distance_cm <= ALERT_VERYFAR_LIMIT) {
        return ALERT_VERYFAR;

    } else {
        return ALERT_SILENT;
    }
}

// simplified update_beeper_alerts function
void update_beeper_alerts(alert_level_t new_level) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check if alert level changed
    if (new_level != beeper.current_level) {
        beeper.current_level = new_level;
        beeper.last_beep_time = 0;
        beeper.is_beeping = false;
        set_beeper_tone(0, false);
    } 
    
    const alert_config_t *config = &alert_configs[beeper.current_level];

    if (beeper.current_level == ALERT_SILENT) {
        if (beeper.is_beeping) {
            set_beeper_tone(0, false);
            beeper.is_beeping = false;
        }
        return;
    } else {

 ESP_LOGI(TAG, "new_level %d", new_level);

       if (new_level ==6){
gpio_set_level(RED_PIN_1, 1); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 
        }
       if (new_level ==5){
gpio_set_level(RED_PIN_1, 1); 
gpio_set_level(BLUE_PIN_1, 1);  
gpio_set_level(GREEN_PIN_1, 0); 
        }
if (new_level ==4 ){
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 1);  
gpio_set_level(GREEN_PIN_1, 0); 
        }
if (new_level ==3 ){
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 1);  
gpio_set_level(GREEN_PIN_1, 1); 
        }

        if (new_level ==2){
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 1); 
        }
        if (new_level ==1){
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 
        }


    }

    if (beeper.is_beeping) {
        if (current_time - beeper.beep_start_time >= config->beep_duration_ms) {    
            set_beeper_tone(0, false);
            beeper.is_beeping = false;
            beeper.last_beep_time = current_time;
        }
    } else {
        if (current_time - beeper.last_beep_time >= config->beep_interval_ms) {
            set_beeper_tone(config->frequency_hz, true);
            beeper.is_beeping = true;
            beeper.beep_start_time = current_time;
        }
    }
}

/*
void system_monitor_task(void *pvParameters) {
    while(1) {
        printf("### Task List ###\n");
        vTaskList();
        printf("### Runtime Stats ###\n");
        vTaskGetRunTimeStats( NULL, 100, NULL, NULL );
        printf("####################\n\n");
        vTaskDelay( pdMS_TO_TICKS( 60000 )); // Log every minute
    }
}
*/

/**
 * @brief Attempts to fully recover the VL53L5CX sensor after it has stopped responding.
 * 
 * @param dev Pointer to the sensor's configuration/device struct.
 * @return true if recovery was successful, false otherwise.
 */
bool vl53l5cx_recoverX(VL53L5CX_Configuration *dev)
{
    bool success = false;
    ESP_LOGW(TAG, "Attempting full recovery of VL53L5CX sensor...");

    // Take the I2C mutex and HOLD IT for the entire recovery process
    if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        // Now we have exclusive access to the I2C bus. The MPU task is blocked.
        
        vl53l5cx_stop_ranging(dev);
        // ... hardware reset ...


        uint8_t status = vl53l5cx_init(dev);
        if (status == 0) {
            status = vl53l5cx_start_ranging(dev);
            if (status == 0) {
                ESP_LOGI(TAG, "VL53L5CX recovery successful!");
                success = true;
            } else {
                ESP_LOGE(TAG, "Recovery failed: vl53l5cx_start_ranging() failed");
            }
        } else {
            ESP_LOGE(TAG, "Recovery failed: vl53l5cx_init() failed");
        }

        // ALWAYS give the mutex back when done.
        xSemaphoreGive(g_i2c_bus_mutex);
    } else {
        ESP_LOGE(TAG, "Recovery failed: Could not obtain I2C bus lock.");
    }

    return success;
}

bool vl53l5cx_recover(VL53L5CX_Configuration *dev) {
    ESP_LOGW(TAG, "Attempting VL53L5CX recovery...");
    
    // Take I2C mutex for recovery operations
    if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take I2C mutex for recovery");
        return false;
    }
    
    uint8_t status;
    uint8_t isAlive;
    
    // Step 1: Stop any ongoing ranging
    ESP_LOGI(TAG, "Stopping ranging...");
    vl53l5cx_stop_ranging(dev);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Step 2: Hardware reset
    ESP_LOGI(TAG, "Performing hardware reset...");
    VL53L5CX_Reset_Sensor(&(dev->platform));
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset to complete
    
    // Step 3: Check if sensor is alive after reset
    ESP_LOGI(TAG, "Checking sensor status after reset...");
    status = vl53l5cx_is_alive(dev, &isAlive);
    if (!isAlive || status) {
        ESP_LOGE(TAG, "Sensor not alive after reset. Status: %d, Alive: %d", status, isAlive);
        xSemaphoreGive(g_i2c_bus_mutex);
        return false;
    }
    
    // Step 4: Re-initialize the sensor
    ESP_LOGI(TAG, "Re-initializing sensor...");
    status = vl53l5cx_init(dev);
    if (status) {
        ESP_LOGE(TAG, "Re-initialization failed with status: %d", status);
        xSemaphoreGive(g_i2c_bus_mutex);
        return false;
    }
    
    // Step 5: Reconfigure the sensor
    ESP_LOGI(TAG, "Reconfiguring sensor...");
    status = vl53l5cx_set_resolution(dev, VL53L5CX_RESOLUTION_4X4);
    if (status) {
        ESP_LOGW(TAG, "Failed to set resolution during recovery: %d", status);
    }
    
    status = vl53l5cx_set_ranging_frequency_hz(dev, 15);
    if (status) {
        ESP_LOGW(TAG, "Failed to set frequency during recovery: %d", status);
    }
    
    // Step 6: Restart ranging
    ESP_LOGI(TAG, "Restarting ranging...");
    status = vl53l5cx_start_ranging(dev);
    if (status) {
        ESP_LOGE(TAG, "Failed to restart ranging: %d", status);
        xSemaphoreGive(g_i2c_bus_mutex);
        return false;
    }
    
    // Release mutex
    xSemaphoreGive(g_i2c_bus_mutex);
    
    ESP_LOGI(TAG, "VL53L5CX recovery successful!");
    return true;
}


// New task function for the sensor loop
void vl53l5cx_reader_task(void *pvParameters) {


    uint32_t minStackBytesRemaining = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Sensor Task: Initial Stack Remaining %d bytes", minStackBytesRemaining);
    static uint32_t checkCounter = 0;
   
   // --- Add a counter for consecutive "not ready" failures ---
    int not_ready_count = 0;
    const int NOT_READY_THRESHOLD = 10; // Trigger recovery after 10 failures (~500ms)

    // gyro stuff
    mpu_data_t current_orientation;
    // Set a default orientation in case we haven't received one yet
    current_orientation.pitch = 0;
    current_orientation.roll = 0;



    sensor_task_params_t *params = (sensor_task_params_t *)pvParameters;
    VL53L5CX_Configuration *Dev = params->dev;
    const uint8_t (*gridLayout)[4] = params->gridLayout;
    alert_level_t *gridAlerts = params->gridAlerts;
    
    uint8_t status, isReady, i;
    VL53L5CX_ResultsData Results;
    uint32_t last_sensor_read = 0;
    const uint32_t SENSOR_READ_INTERVAL = 50; // 1ms
    int loop = 0;
    uint16_t primary_distance_total = 0;

   // Static arrays for grid layout and alerts
    static const uint8_t gridTop[4] = {
        0, 4, 8, 12
    };
    int gridTop_arr_size = sizeof(gridTop)/sizeof(gridTop[0]);


static const uint8_t gridBottom[4] = {
        3, 7, 11, 15
    };

int gridBottom_arr_size = sizeof(gridBottom)/sizeof(gridBottom[0]);

  // Static arrays for grid layout and alerts
    static const uint8_t gridLeft[2] = {
        1, 2
    };
static const uint8_t gridRight[2] = {
         13, 14
    };

static const uint8_t gridCentre[8] = {
         1,2,5,6,9,10,13,14
    };


     // --- NEW: Add a state variable to track the last sent alert level ---
alert_level_t last_sent_ws2812_level = (alert_level_t)-1; // Init to invalid value

static led_state_t last_sent_led_state{}; 


    // Start ranging
 //   status = vl53l5cx_start_ranging(Dev);


if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
  
         status = vl53l5cx_check_data_ready(Dev, &isReady);



    if (status) {
        ESP_LOGE(TAG, "Failed to start ranging");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Sensor task started successfully");

    while(1) {


#ifdef useAccelerometer

       if (xQueueReceive(mpu_data_queue, &current_orientation, 0) == pdPASS) {
            // New orientation data is now stored in 'current_orientation'
        }


            if (current_orientation.pitch > 45.0) {
                 ESP_LOGI("VL53_TASK", "Device is pointed UPWARDS (Pitch: %.1f). Adjusting logic.", current_orientation.pitch);
                 // e.g., Treat the top row of the sensor as the "forward" direction
            } else if (current_orientation.pitch < -45.0) {
                 ESP_LOGI("VL53_TASK", "Device is pointed DOWNWARDS (Pitch: %.1f). Ignoring floor.", current_orientation.pitch);
                 // e.g., Filter out readings from the bottom rows
            } else {
                 // Device is relatively level, use default logic
            }

#endif


bool topAlert = false;
bool bottomAlert = false;

        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (current_time - last_sensor_read >= SENSOR_READ_INTERVAL) {
            last_sensor_read = current_time;

   

            if(isReady) {

  not_ready_count = 0; // Reset counter on success

                vl53l5cx_get_ranging_data(Dev, &Results);

                primary_distance_total = 0;




                for(i = 0; i < 16; i++) {
                    for(int j = 0; j < VL53L5CX_NB_TARGET_PER_ZONE; j++) {
                        uint16_t idx = VL53L5CX_NB_TARGET_PER_ZONE * i + j;

                        uint16_t primary_distance = 0;
                        primary_distance = Results.distance_mm[idx]/10;

                        gridAlerts[i] = get_alert_level(primary_distance);

//printf("grid %d:%d \n", i, gridAlerts[i]);



if ((i % 4) == 0) {

     if( gridAlerts[i] > 3  ){
        topAlert = true;
     }

    }



if ((i % 4) == 3) {
 if( gridAlerts[i] > 3 ){ 
bottomAlert = true;
 }
}


uint8_t remainder = i % 4;
if (remainder == 1 || remainder == 2) {
    // 'i' is in gridCentre.
     primary_distance_total += primary_distance;
}


                    }
                } // for every square

  xSemaphoreGive(g_i2c_bus_mutex);


                // Calculate average and update alerts
                int average_distance = primary_distance_total/8;
                alert_level_t alert_level_average = get_alert_level(average_distance);


ESP_LOGI(TAG, "average tot:%d avg:%d", primary_distance_total, average_distance);
update_beeper_alerts(alert_level_average);


            // --- Create and populate a single LED state object ---
 //           led_state_t desired_led_state{};
            const alert_config_t *config = &alert_configs[alert_level_average];
    
#ifdef useRGBLed
 // --- NEW: Create and populate a single, simple LED state object ---
    led_state_t desired_led_state{}; // Zero-initialize


    // Map color from the config table
    switch (config->led_color) {
        case LedColor::BLACK:  desired_led_state.r = 0; desired_led_state.g = 0; desired_led_state.b = 0; break;
        case LedColor::RED:    desired_led_state.r = 255; break;
        case LedColor::ORANGE: desired_led_state.r = 255; desired_led_state.g = 140; break;
        case LedColor::YELLOW: desired_led_state.r = 255; desired_led_state.g = 255; break;
        case LedColor::GREEN:  desired_led_state.g = 255; break;
        case LedColor::BLUE:   desired_led_state.b = 255; break;
        case LedColor::WHITE:  desired_led_state.r = 255; desired_led_state.g = 255; desired_led_state.b = 255; break;
        default: break; // For BLACK or OFF, r,g,b will remain 0
    }
    desired_led_state.brightness = config->led_strength_val;

ESP_LOGI(TAG, "LED trying manual light %d %d %d %u", desired_led_state.r, desired_led_state.g, desired_led_state.b, desired_led_state.brightness);
    rgb_led_set_color_with_brightness(desired_led_state.r, desired_led_state.g, desired_led_state.b, 1.0); // 50% brightness red


    // Part 2: Populate the GPIO part of the state
    desired_led_state.top_led_on = topAlert;
    desired_led_state.bottom_led_on = bottomAlert;

     if (desired_led_state != last_sent_led_state) {

 

        ESP_LOGI(TAG, "LED state change detected. Sending update.");
    // Part 3: Send the single, complete state object to the queue
    // We use xQueueOverwrite to ensure the LED task always has the freshest data.

    //  commented out for debug
   xQueueOverwrite(led_state_queue, &desired_led_state);

// Update our record of the last sent state.
                last_sent_led_state = desired_led_state;
     } else {

ESP_LOGI(TAG, "LED state no change detected");
     }
#endif

                // Print grid visualization
                for (int row = 0; row <= 3; row++) {

                        if (row == 0) {
                            printf("\n-\n");
                        }

                    for (int col = 3; col >= 0; col--) {
                        alert_level_t level = gridAlerts[gridLayout[row][col]];
                        
                        switch(level) {
                            

                            case alert_level_t::ALERT_SILENT:
                                printf("%2d", 0);
                                break;
                            case alert_level_t::ALERT_VERYFAR:
                                printf("%2d", 1);
                                break;
                            case alert_level_t::ALERT_FAR:
                                printf("%2d", 1);
                                break;
                            case alert_level_t::ALERT_MEDIUMFAR:
                                printf("%2d", 2);
                                break;
                            case alert_level_t::ALERT_MEDIUM:
                                printf("%2d", 3);
                                break;
                            case alert_level_t::ALERT_CLOSE:
                                printf("%2d", 4);
                                break;
                            case alert_level_t::ALERT_IMMEDIATE:
                                printf("%2d", 5);
                                break;
                        }

                        if (col > 0) {
                            printf(" | ");
                        }
                        
                        if (col == 0) {
                            printf("\n");
                        }
                    }

                   
                }

            

        
              //  loop++;
            } else {
           not_ready_count++;
            ESP_LOGW(TAG, "Sensor not ready or comms error. Count: %d/%d", not_ready_count, NOT_READY_THRESHOLD);
            vTaskDelay(pdMS_TO_TICKS(50));

            if (not_ready_count >= NOT_READY_THRESHOLD) {
                ESP_LOGE(TAG, "Not ready threshold reached. Attempting recovery...");

                   xSemaphoreGive(g_i2c_bus_mutex);

                // Call the new, comprehensive recovery function
                if (vl53l5cx_recover(Dev)) {
                    // Recovery was successful, reset the counter and continue.
                    not_ready_count = 0;
                    vTaskDelay(pdMS_TO_TICKS(50));
                } else {
                    // Catastrophic failure. The sensor couldn't be recovered.
                    // We can either try again after a long delay or stop the task.
                    ESP_LOGE(TAG, "Catastrophic recovery failure. Halting task.");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    //vTaskDelete(NULL); // Or vTaskDelay for a long time...
                }
            }

            }
VL53L5CX_WaitMs(&(Dev->platform), 5);
            // printf("-------------------\n");
        } // SENSOR_READ_INTERVAL passed
           // Periodic Stack Check (e.g., every 10 iterations)
       
        checkCounter++;

 
        if( checkCounter % 10 == 0 ) {
            minStackBytesRemaining = uxTaskGetStackHighWaterMark(NULL);
      //      ESP_LOGI(TAG, "Sensor Task Stack : %d bytes left", minStackBytesRemaining);
            if( minStackBytesRemaining < 100 ){
                ESP_LOGW(TAG, "Sensor Task Stack Warning: %d bytes left", minStackBytesRemaining);
            }
        }
    


        // Small delay to prevent watchdog issues and allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(50));

//esp_task_wdt_feed();

     } // while 1 loop end

    } // while 1 loop end

    // Cleanup (though this won't be reached in normal operation)
    vl53l5cx_stop_ranging(Dev);
    set_beeper_tone(0, false);
    vTaskDelete(NULL);
}

void find_reset_reason( int resetreason){

   // SW_CPU_RESET

    if (resetreason == 8) {
        ESP_LOGI(TAG, "Reset ESP_RST_DEEPSLEEP");
     //       break;
    
    //  return();
    }
    
              if (resetreason == 3) {
                 ESP_LOGI(TAG, "Reset ESP_RST_SW");
        //       break;
     
           //  return();
          }
     
              if (resetreason == 4 || resetreason == 14) {
                 ESP_LOGI(TAG, "Reset ESP_RST_PANIC or ESP_RST_CPU_LOCKUP");
       //       break;
     
           //  return();
          }
     
              if (resetreason == 5) {
                 ESP_LOGI(TAG, "Reset ESP_RST_INT_WDT");
       //       break;
     
           //  return();
          }
     
              if (resetreason == 9 || resetreason == 14) {
                 ESP_LOGI(TAG, "Reset ESP_RST_BROWNOUT or ESP_RST_PWR_GLITCH");
            //       break;
     
           //  return();
          }
     
          if (resetreason == 6) {
             ESP_LOGI(TAG, "Reset ESP_RST_TASK_WDT");
      
       //  return();
      }
     
     
  

  }
 

extern "C" void app_main(void)
{
 
     esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,    // Bitmask of all cores
        .trigger_panic = false,
    };

gpio_set_direction(RED_PIN, GPIO_MODE_OUTPUT);
gpio_set_direction(BLUE_PIN, GPIO_MODE_OUTPUT);
gpio_set_direction(GREEN_PIN, GPIO_MODE_OUTPUT);
gpio_set_level(RED_PIN, 0); 
gpio_set_level(BLUE_PIN, 0);  
gpio_set_level(GREEN_PIN, 0); 


gpio_set_direction(RED_PIN_1, GPIO_MODE_OUTPUT);
gpio_set_direction(BLUE_PIN_1, GPIO_MODE_OUTPUT);
gpio_set_direction(GREEN_PIN_1, GPIO_MODE_OUTPUT);
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 


    uint32_t last_sensor_read = 0;
    i2c_port_t i2c_port = I2C_NUM_1;
 

   i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = i2c_port,
        .sda_io_num = GPIO_NUM_1,
        .scl_io_num = GPIO_NUM_2,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags {
            .enable_internal_pullup = true,
            .allow_pd = false,
        }
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

#ifdef useRGBLed
    // Create the mutex to protect the LED strip driver
    led_strip_mutex = xSemaphoreCreateMutex();
    if (led_strip_mutex == NULL) {
        ESP_LOGE("MAIN", "Failed to create LED strip mutex!");
        return; // Cannot continue
    }
#endif

        g_i2c_bus_mutex = xSemaphoreCreateMutex();
    if (g_i2c_bus_mutex == NULL) {
        ESP_LOGE("MAIN", "Fatal: Failed to create I2C bus mutex!");
        return;
    }

#ifdef useAccelerometer
        mpu_data_queue = xQueueCreate(5, sizeof(mpu_data_t));
#endif

led_state_queue = xQueueCreate(1, sizeof(led_state_t)); // Correct queue
ESP_LOGI("MAIN", "Mutex and Queues created.");

#ifdef useRGBLed
configure_led();
#endif

//rgb_led_init();
vTaskDelay(pdMS_TO_TICKS(100));

#ifdef useRGBLed

    // 3. Try to set the LED as testto RED 50%.");
        rgb_led_set_color_with_brightness(255, 0, 0, 0.5); // 50% brightness red
 vTaskDelay(pdMS_TO_TICKS(500));
    rgb_led_set_color_with_brightness(0, 255, 0, 0.5); // 50% brightness red
 vTaskDelay(pdMS_TO_TICKS(500));
    rgb_led_set_color_with_brightness(0, 0, 255, 0.5); // 50% brightness red
 vTaskDelay(pdMS_TO_TICKS(500));
#endif

   // Initialize piezo beeper
    esp_err_t ret = init_piezo_beeper();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize piezo beeper");
        return;
    }
   ESP_LOGI(TAG, "initialized piezo beeper");

// --- 1. Declare necessary variables ---
uint8_t status, isAlive;
VL53L5CX_Configuration Dev; // Your main driver struct

// --- 2. Define the I2C device configuration for the VL53L5CX ---
i2c_device_config_t vl53_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    // Use the raw 7-bit address. The default is 0x29.
    // The driver handles the R/W bit, so no shifting is needed.
    .device_address = 0x29, 
    .scl_speed_hz = VL53L5CX_MAX_CLK_SPEED,
};

// --- 3. Add the device to the bus. This creates the handle. ---
// The handle is stored inside your Dev struct, ready for the driver to use.
ESP_LOGI(TAG, "Adding VL53L5CX sensor to I2C bus...");
i2c_master_bus_add_device(bus_handle, &vl53_dev_cfg, &Dev.platform.handle);

// --- 4. Now that communication is possible, perform hardware reset and init ---
ESP_LOGI(TAG, "Resetting VL53L5CX sensor...");

Dev.platform.reset_gpio = GPIO_NUM_5;
VL53L5CX_Reset_Sensor(&(Dev.platform));

// --- 5. Check if the sensor is alive (this uses the handle created in step 3) ---
ESP_LOGI(TAG, "Checking if VL53L5CX is alive...");
status = vl53l5cx_is_alive(&Dev, &isAlive);
if (!isAlive || status) {
    ESP_LOGE(TAG, "VL53L5CX not detected at address 0x29! Check wiring and address.");
    return; // Stop here if sensor is not found
}
ESP_LOGI(TAG, "VL53L5CX is alive!");

// --- 6. Initialize the sensor (this also uses the handle created in step 3) ---
ESP_LOGI(TAG, "Initializing VL53L5CX ULD...");
status = vl53l5cx_init(&Dev);
if (status) {
    ESP_LOGE(TAG, "VL53L5CX ULD Loading failed, error code: %d", status);
    return;
}

ESP_LOGI(TAG, "VL53L5CX ULD ready! (Version: %s)", VL53L5CX_API_REVISION);

#ifdef useAccelerometer

// --- MPU9250 Initialization ---
    i2c_master_dev_handle_t mpu9250_dev_handle;
    i2c_device_config_t mpu9250_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_I2C_ADDRESS, // 0x68
        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu9250_dev_cfg, &mpu9250_dev_handle));
    ESP_LOGI("I2C", "Device MPU9250 added to bus");
    
    // IMPORTANT: Wake up the MPU9250 from sleep mode
    ESP_LOGI("MPU9250", "Waking up sensor...");
    ESP_ERROR_CHECK(i2c_master_transmit(
        mpu9250_dev_handle, 
        (const uint8_t[]){ MPU9250_PWR_MGMT_1_REG_ADDR, 0x00 }, // Write 0 to PWR_MGMT_1
        2, 
        pdMS_TO_TICKS(100)
    ));

    ESP_LOGI(TAG, "MPU9250 Initialized.");


    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for the sensor to stabilize

#endif

    ESP_LOGI("MAIN", "Initialization complete. Tasks are running.");

 
    // Prepare task parameters
    static sensor_task_params_t task_params = {
        .dev = &Dev,
        .gridLayout = gridLayout,
        .gridAlerts = gridAlerts
    };

 // --- 5. Create and Launch All Tasks ---
    ESP_LOGI("MAIN", "Creating tasks...");

    #ifdef useRGBLed
    ESP_LOGI("MAIN", "Creating task for RGB LED");
    // Create the LED task. It's now safe to start.
    xTaskCreate(led_control_task, "LED Control", 4096, NULL, 4, NULL);
#endif

    // Create sensor and data processing tasks
#ifdef useAccelerometer
ESP_LOGI("MAIN", "Creating task for Accelerometer");
    xTaskCreate(mpu9250_reader_task, "MPU Reader", 4096, mpu9250_dev_handle, 5, NULL);
#endif

    xTaskCreate(data_processor_task, "Data Processor", 4096, NULL, 3, NULL);
    xTaskCreate(vl53l5cx_reader_task, "VL53L5CX Reader", 8192, &task_params, 5, NULL);


}
