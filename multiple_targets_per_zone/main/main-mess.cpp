#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l5cx_api.h"

///#include "esp_vuart_new.h"  // For reliable serial output

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"

#include "led_strip.h"
#include "led_strip_rmt.h"
#include "math.h"


#define WS2812_GPIO 48
#define WS2812_LED_COUNT 1

// Detection parameters
#define MOVING_AVERAGE_WINDOW_SIZE 15

static led_strip_handle_t led_strip = NULL;
#define PIEZO_BEEPER_PIN GPIO_NUM_38
#define BEEPER_LEDC_TIMER LEDC_TIMER_0
#define BEEPER_LEDC_MODE LEDC_LOW_SPEED_MODE
#define BEEPER_LEDC_CHANNEL LEDC_CHANNEL_0
#define BEEPER_DUTY_RESOLUTION LEDC_TIMER_10_BIT

// Audio alert parameters
#define BEEP_DUTY_CYCLE         512  // 50% duty cycle for clear tone


// Alert levels
typedef enum {
    ALERT_SILENT = 0,
    ALERT_FAR,          // 300-600cm: Low freq, slow beeps
    ALERT_MEDIUMFAR,
    ALERT_MEDIUM,       // 150-300cm: Medium freq, medium beeps  
    ALERT_CLOSE,        // 50-150cm: High freq, fast beeps
    ALERT_IMMEDIATE     // 0-50cm: Very high freq, rapid beeps
} alert_level_t;

// LED color enum
typedef enum {
    BLACK,
    BLUE,
    GREEN,
    YELLOW,
    ORANGE,
    RED,
    CYAN,
    MAGENTA,
    WHITE
} LedColor;

typedef struct {
    uint32_t frequency_hz;      // Beep frequency in Hz
    uint32_t beep_duration_ms;  // Duration of each beep in ms
    uint32_t beep_interval_ms;  // Time between beeps in ms
    const char* description;     // Description of the alert level
    LedColor led_color;          // LED color for the alert
    int led_blink_times;         // Number of LED blinks
    int led_blink_delay_ms;      // Delay between LED blinks in ms
    float led_strength_val;      // LED brightness level (1.0 = full brightness)
} alert_config_t;

// Beeper state
typedef struct {
    alert_level_t current_level;
    bool is_beeping;
    uint32_t beep_start_time;
    uint32_t last_beep_time;
    uint32_t beep_count;
} beeper_state_t;

// Global variables
static const char *TAG = "WALKING_AID";

static const alert_config_t alert_configs[] = {
    {0,    0,   0,    "Silent",      LedColor::GREEN,   1, 100, 0.1}, // ALERT_SILENT
    {800,  50,  50, "Far",           LedColor::GREEN,   1, 200, 0.3}, // ALERT_FAR
    {1000, 50,  50, "MediumFar",     LedColor::YELLOW,  1, 200, 0.4}, // ALERT_MEDIUMFAR
    {1200, 50,  80, "Medium",        LedColor::ORANGE,  1, 300, 0.5}, // ALERT_MEDIUM
    {1800, 50,  120,  "Close",       LedColor::RED,     1, 200, 0.7}, // ALERT_CLOSE
    {2500, 70, 200,  "Immediate",    LedColor::WHITE,   2, 100, 1}   // ALERT_IMMEDIATE
};

alert_level_t gridAlerts[16];
volatile beeper_state_t beeper = {ALERT_SILENT, false, 0, 0, 0};
uint8_t isReady = 0;
VL53L5CX_ResultsData Results;
VL53L5CX_Configuration Dev = {0};
uint16_t primary_distance_total = 0;

// Function declarations
static void i2c_init_master(void);
void rgb_led_init();
esp_err_t init_piezo_beeper(void);
void rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b);
void rgb_led_blink(LedColor color, int times, int ms_delay, float strength_val);
void rgb_led_off();
void rgb_led_set_color_with_brightness(uint8_t r, uint8_t g, uint8_t b, float brightness);
void set_beeper_tone(uint32_t frequency_hz, bool enable);
alert_level_t get_alert_level(uint16_t distance_cm);
void update_beeper_alerts(alert_level_t new_level);
const char* led_color_to_string(LedColor color);

VL53L5CX_Configuration dev_config_val;

beeper_state_t beeper;

// Function definitions
void rgb_led_init() {
    esp_err_t err;

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,  // 10 MHz resolution
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        },
    };

    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = WS2812_LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

    if (err != ESP_OK || led_strip == NULL) {
        ESP_LOGE("LED", "LED strip init failed: %s", esp_err_to_name(err));
    } else {
        ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
    }
}

void rgb_led_off() {
    esp_err_t err = led_strip_clear(led_strip);

    if (err != ESP_OK || led_strip == NULL) {
        ESP_LOGE("LED", "rgb_led_off failed: %s", esp_err_to_name(err));
    }
}

void rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b) {

    esp_err_t err;

    if (led_strip == nullptr) {
        ESP_LOGE("LED", "led_strip is NULL, skipping rgb_led_set_color");
        return;
    }

    err = led_strip_set_pixel(led_strip, 0, r, g, b);
    if (err != ESP_OK || led_strip == NULL) {
        ESP_LOGE("LED", "led_strip_set_pixel failed: %s", esp_err_to_name(err));
    } else {
        err = led_strip_refresh(led_strip);
        if (err != ESP_OK || led_strip == NULL) {
            ESP_LOGE("LED", "led_strip_refresh failed: %s", esp_err_to_name(err));
        }
    }
}

uint8_t gamma_correct(uint8_t val) {
    return (uint8_t)(std::powf(val / 255.0f, 2.2f) * 255.0f);
}


void rgb_led_set_color_with_brightness(uint8_t r, uint8_t g, uint8_t b, float brightness) {
    if (brightness < 0.0f) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;

    uint8_t scaled_r = gamma_correct((uint8_t)(r * brightness));
    uint8_t scaled_g = gamma_correct((uint8_t)(g * brightness));
    uint8_t scaled_b = gamma_correct((uint8_t)(b * brightness));

    led_strip_set_pixel(led_strip, 0, scaled_r, scaled_g, scaled_b);
    led_strip_refresh(led_strip);
}


void rgb_led_blink(LedColor color, int times, int ms_delay, float strength_val) {
    uint8_t r = 0, g = 0, b = 0;

    switch (color) {
        case RED:      r = 255; g = 0;   b = 0;   break;
        case ORANGE:   r = 255; g = 140; b = 0;   break;
        case GREEN:    r = 0;   g = 255; b = 0;   break;
        case BLUE:     r = 0;   g = 0;   b = 255; break;
        case YELLOW:   r = 255; g = 255; b = 0;   break;
        case CYAN:     r = 0;   g = 255; b = 255; break;
        case MAGENTA:  r = 180; g = 100; b = 255; break;
        case WHITE:    r = 255; g = 255; b = 255; break;
        case BLACK:    r = 0;   g = 0;   b = 0;   break;
        default:       r = 0;   g = 0;   b = 0;   break;
    }

    for (int i = 0; i < times; ++i) {
        rgb_led_set_color_with_brightness(r, g, b, strength_val);
        vTaskDelay(pdMS_TO_TICKS(ms_delay));
        rgb_led_off();
        vTaskDelay(pdMS_TO_TICKS(ms_delay));
    }
}

const char* led_color_to_string(LedColor color) {
    switch (color) {
        case BLUE:        return "BLUE";
        case BLACK:        return "BLACK";
        case GREEN:        return "GREEN";
        case YELLOW:       return "YELLOW";
        case ORANGE:       return "ORANGE";
        case RED:          return "RED";
        case CYAN:         return "CYAN";
        case MAGENTA:      return "MAGENTA";
        case WHITE:        return "WHITE";
        default:           return "UNKNOWN_COLOR";
    }
}

esp_err_t init_piezo_beeper(void) {
    ledc_timer_config_t timer_config = {
        .speed_mode = BEEPER_LEDC_MODE,
        .duty_resolution = BEEPER_DUTY_RESOLUTION,
        .timer_num = BEEPER_LEDC_TIMER,
        .freq_hz = 1000,  // Default frequency, will be changed dynamically
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .gpio_num = PIEZO_BEEPER_PIN,
        .speed_mode = BEEPER_LEDC_MODE,
        .channel = BEEPER_LEDC_CHANNEL,
        .timer_sel = BEEPER_LEDC_TIMER,
        .duty = 0,  // Start silent
        .hpoint = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    ESP_LOGI(TAG, "Piezo beeper initialized on GPIO%d", PIEZO_BEEPER_PIN);
    return ESP_OK;
}

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

alert_level_t get_alert_level(uint16_t distance_cm) {
    if (distance_cm <= 40) {  // IMMEDIATE
        return ALERT_IMMEDIATE;
    } else if (distance_cm <= 80) {  // CLOSE
        return ALERT_CLOSE;
    } else if (distance_cm <= 100) {  // MEDIUM
        return ALERT_MEDIUM;
    } else if (distance_cm <= 150) {  // MEDIUMFAR
        return ALERT_MEDIUMFAR;
    } else if (distance_cm <= 200) {  // FAR
        return ALERT_FAR;
    } else {
        return ALERT_SILENT;
    }
}


void update_beeper_alerts(alert_level_t new_level) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check if alert level changed
    if (new_level != beeper.current_level) {
        beeper.current_level = new_level;
        beeper.last_beep_time = 0;  // Reset timing to start new pattern immediately
        beeper.is_beeping = false;
        set_beeper_tone(0, false);  // Stop current beep

        ESP_LOGI(TAG, "🔊 Alert level changed to: %s", alert_configs[new_level].description);

        const alert_config_t *config = &alert_configs[beeper.current_level];

        // Blink LED
        rgb_led_blink(config->led_color, config->led_blink_times, config->led_blink_delay_ms, config->led_strength_val);
        ESP_LOGI(TAG, "LED Color at update_beeper_alerts: %s (%d)", 
                 led_color_to_string(config->led_color),
                 (int)config->led_color);

        if (beeper.current_level == ALERT_SILENT) {
            rgb_led_off();
        }
    }

    // Handle beeping pattern
    if (!beeper.is_beeping && beeper.current_level < ALERT_IMMEDIATE) {
        const alert_config_t *config = &alert_configs[beeper.current_level];
        set_beeper_tone(config->frequency_hz, true);
        beeper.is_beeping = true;
        beeper.beep_start_time = current_time;
        beeper.last_beep_time = current_time;
        beeper.beep_count = 0;
    }

    // Check if current beep should end
    if (beeper.is_beeping) {
        if (current_time - beeper.beep_start_time >= alert_configs[beeper.current_level].beep_duration_ms) {    
            set_beeper_tone(0, false);
            beeper.is_beeping = false;
            beeper.last_beep_time = current_time;
            beeper.beep_count++;
        } else if (current_time - beeper.last_beep_time >= alert_configs[beeper.current_level].beep_interval_ms && !beeper.is_beeping) {
            set_beeper_tone(alert_configs[beeper.current_level].frequency_hz, true);
            beeper.beep_start_time = current_time;
            beeper.last_beep_time = current_time;
            beeper.is_beeping = true;
            beeper.beep_count++;
        }
    }

    // Stop beeping after 3 beeps for immediate level
    if (beeper.current_level == ALERT_IMMEDIATE && beeper.beep_count >= 3) {
        beeper.is_beeping = false;
        set_beeper_tone(0, false);
    } else if (beeper.current_level != ALERT_IMMEDIATE && beeper.beep_count >= 2) {
        beeper.is_beeping = false;
        set_beeper_tone(0, false);
    }
}

/// Initialize I2C master
static void i2c_init_master(void) {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_1,
        .scl_io_num = GPIO_NUM_2,
        .baudrate = 400000,
        .flags = {
            .master = {
                .timeout = 1000,
                .huffman = 1
            }
        }
    };
    ESP_ERROR_CHECK(i2c_param_init_default(&i2c_conf));
    ESP_ERROR_CHECK(i2c_master_init(I2C_NUM_1, &i2c_conf));

    // Reset sensor by toggling reset line
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_5, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(GPIO_NUM_5, 1);
}

void process_grid_data() {
    // Process 16 zone distances
    for(int i = 0; i < 16; i++) {
        int distance = Results.distance_mm[i] / 10;
        alert_level_t level = get_alert_level(distance);
        gridAlerts[i] = level;
    }
}
// Task function for continuous ranging
static void range_sensor_task(void *arg) {
    const TickType_t delay = pdMS_TO_TICKS(100);  // Read every 100ms
    uint32_t last_reading = 0;

    while(1) {
        uint32_t current_time = xTaskGetTickCount();
        if (current_time - last_reading >= delay) {
            last_reading = current_time;

            // Check for new data
            uint8_t is_data_ready;
            ESP_ERROR_CHECK(vl53l5cx_check_data_ready(&dev_config_val, &is_data_ready));

            if (is_data_ready) {
                ESP_ERROR_CHECK(vl53l5cx_get_ranging_data(&dev_config_val, &Results));

                // Process grid data
                process_grid_data();
            }
        }
    }
}

/*
static void task_main_loop(void *arg) {
    const uint32_t SENSOR_READ_INTERVAL_MS = 100; // 10Hz
    uint32_t last_sensor_read = 0;

    ESP_ERROR_CHECK(vl53l5cx_start_ranging(&Dev));

    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Read sensor at configured interval
        if (current_time - last_sensor_read >= SENSOR_READ_INTERVAL_MS) {
            last_sensor_read = current_time;
            
            // Check if new data is available
            ESP_ERROR_CHECK(vl53l5cx_check_data_ready(&Dev, &isReady));
            
            if (isReady) {
                // Get the ranging data
                ESP_ERROR_CHECK(vl53l5cx_get_ranging_data(&Dev, &Results));
                
                primary_distance_total = 0;
                
                // Reset grid alert levels
                for(int i = 0; i < 16; i++) {
                    gridAlerts[i] = ALERT_SILENT;
                }
                
                // Process all 16 zones
                for(int idx = 0; idx < 16; idx++) {
                    if (idx % 4 == 0) {
                        // New row, print row info if any changes
                    }
                    
                    uint16_t distance_mm = Results.distance_mm[idx] / 10;
                    if (distance_mm <= 3000) {  // Only process reasonable distances
                        gridAlerts[idx] = get_alert_level(distance_mm);
                        primary_distance_total += distance_mm;
                        
                        // Optional: Print alert level to console
                        ESP_LOGD(TAG, "Zone %d: %d cm (%s)", idx, distance_mm, 
                                 alert_configs[gridAlerts[idx]].description);
                        
                        if (idx == 0) {
                            ESP_LOGI(TAG, "Grid initialized:");
                        }
                    }
                    
                    update_beeper_alerts(gridAlerts[idx]);
                }
                
                // Calculate average distance
                int average_level = primary_distance_total / 16;
                update_beeper_alerts(get_alert_level(average_level));
                
                // Print grid visualization
                char grid_string[65];
                grid_string[0] = '\0';
                
                for(int row = 3; row >= 0; row--) {
                    for(int col = 0; col < 4; col++) {
                        int index = row * 4 + col;
                        char cell[4];

                               // Print the alert level with spaces and pipes for visual clarity
        switch(gridAlerts[index]) {
            case alert_level_t::ALERT_SILENT: {
                printf("%2d", 0);
                break;
            }
            case alert_level_t::ALERT_FAR: {
                printf("%2d", 1);
                break;
            }
            case alert_level_t::ALERT_MEDIUMFAR: {
                printf("%2d", 2);
                break;
            }

            case alert_level_t::ALERT_MEDIUM: {
                printf("%2d", 3);
                break;
            }
            case alert_level_t::ALERT_CLOSE: {
                printf("%2d", 4);
                break;
            }
            case alert_level_t::ALERT_IMMEDIATE: {
                printf("%2d", 5);
                break;
            }
        }
 strcat(grid_string, cell);



                    }
                    strcat(grid_string, "\n");
                }
                
                ESP_LOGI(TAG, "Grid Visualization:\n%s", grid_string);
                
                vTaskDelay(pdMS_TO_TICKS(100 - SENSOR_READ_INTERVAL_MS));
            }
        }
    }
    
    vl53l5cx_stop_ranging(&Dev);
    set_beeper_tone(0, false);
    rgb_led_off();
    vTaskDelete(NULL);
}
*/

extern "C" void app_main(void) {
    // Initialize RGB LED strip
    rgb_led_init();
    rgb_led_off();  // Initialize to off

    // Initialize piezo beeper
    esp_err_t ret = init_piezo_beeper();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize piezo beeper");
        return;
    }

      // Initialize I2C master
    i2c_init_master();
    
    // Initialize VL53L5X sensor
    esp_err_t ret = vl53l5cx_init(&dev_config_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VL53L5X init failed");
        rgb_led_blink(LedColor::RED, 1, 500, 1.0);
        return;
    }

    ESP_LOGI(TAG, "VL53L5X Initialized");

ret = vl53l5cx_start_ranging(&dev_config_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VL53L5X start ranging failed");
        return;
    }

  // Show initialization
    rgb_led_blink(LedColor::RED, 1, 100, 1.0);
    vTaskDelay(pdMS_TO_TICKS(500));
    rgb_led_off();

    /* (Optional) Set new I2C address if needed */
    // ESP_ERROR_CHECK(vl53l5cx_set_i2c_address(&Dev, 0x2A));

// Create a task for continuous ranging
    xTaskCreate(
        range_sensor_task,  // Name of the task function
        "RangeSensorTask",  // Human-readable name
        4096,              // Stack size
        NULL,              // Task parameters
        5,                 // Priority
        NULL               // Task handle
    );

    /* Check if VL53L5CX sensor is connected */
    /*
    uint8_t isAlive = 0;
    ESP_ERROR_CHECK(vl53l5cx_is_alive(&Dev, &isAlive));
    if (!isAlive) {
        ESP_LOGE(TAG, "VL53L5CX not detected");
        rgb_led_blink(LedColor::RED, 2, 200, 1.0);
        return;
    }
*/

    /* Init VL53L5CX sensor */
    /*
    ESP_ERROR_CHECK(vl53l5cx_init(&Dev));

    ESP_LOGI(TAG, "VL53L5CX ULD ready ! (Version : %s)", VL53L5CX_API_REVISION);

    // Setup for 4x4 grid
    const uint8_t gridLayout[4][4] = {
        {0, 1, 2, 3},  // Bottom row
        {4, 5, 6, 7},  // Middle bottom row
        {8, 9, 10, 11}, // Middle top row
        {12, 13, 14, 15} // Top row
    };
*/

  /*
    // Simple animation to show the top and bottom of the grid
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            rgb_led_set_color_with_brightness(0, 0, 0, 1.0);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    rgb_led_blink(LedColor::GREEN, 2, 200, 1.0);
*/

    // Create a task for the main loop
    //xTaskCreate(task_main_loop, "SensorTask", 8192, NULL, 5, NULL);

    // Main task can go to sleep or do other tasks
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Main task running...");
        rgb_led_blink(LedColor::BLUE, 3, 300, 0.5);
    }
}
