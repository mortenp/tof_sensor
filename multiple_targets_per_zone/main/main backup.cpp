#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l5cx_api.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"

////// LED
 #include "led_strip.h"
 #include "led_strip_rmt.h"
 #include "math.h"


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

 int ALERT_IMMEDIATE_LIMIT = 40;
  int ALERT_CLOSE_LIMIT  = 80;
  int ALERT_MEDIUM_LIMIT  = 100;
   int ALERT_MEDIUMFAR_LIMIT  = 150;
  int ALERT_FAR_LIMIT  = 200;
#define HYSTERESIS 15  // cm

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
         .max_leds = WS2812_LED_COUNT, // is set to 1
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
       //  ESP_LOGI("LED", "LED strip init successful: %s", esp_err_to_name(err));
     ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
     }
 }
 
 enum class LedColor {
     OFF,
     RED,
     ORANGE,
     GREEN,
     BLUE,
     YELLOW,
     CYAN,
     MAGENTA,
     WHITE,
     BLACK
 };
 
 const char* led_color_to_string(LedColor color) {
    switch (color) {
        case LedColor::BLACK:    return "BLACK";
        case LedColor::GREEN:    return "GREEN";
        case LedColor::YELLOW:   return "YELLOW";
        case LedColor::ORANGE:   return "ORANGE";
        case LedColor::RED:      return "RED";
        case LedColor::WHITE:    return "WHITE";
        case LedColor::BLUE:    return "BLUE";
        default:                return "UNKNOWN_COLOR";
    }
}

 void rgb_led_off() {
     esp_err_t  err =  led_strip_clear(led_strip);
 
     if (err != ESP_OK || led_strip == NULL) {
         ESP_LOGE("LED", "rgb_led_off failed: %s", esp_err_to_name(err));
     } else {
   //      ESP_LOGI("LED", "rgb_led_off successful: %s", esp_err_to_name(err));
  //   ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
     }
 
 }
 
 
 void rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b) {
     esp_err_t  err ;
 
     if (led_strip == nullptr) {
         ESP_LOGE("LED", "led_strip is NULL, skipping rgb_led_set_color");
         return;
     }
 
       err =  led_strip_set_pixel(led_strip, 0, r, g, b);
       if (err != ESP_OK || led_strip == NULL) {
         ESP_LOGE("LED", "led_strip_set_pixel failed: %s", esp_err_to_name(err));
     } else {
        // ESP_LOGI("LED", "led_strip_set_pixel successful: %s", esp_err_to_name(err));
  //   ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
     }
     
     /*
     if (led_strip == nullptr) {
         ESP_LOGE("LED", "led_strip is NULL, skipping rgb_led_set_color");
         return;
     }
 */
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
         rgb_led_set_color_with_brightness(r, g, b, strength_val);
         vTaskDelay(pdMS_TO_TICKS(ms_delay));
         rgb_led_off();
         vTaskDelay(pdMS_TO_TICKS(ms_delay));
     }
 }
 


typedef enum {
    ALERT_SILENT = 0,
    ALERT_FAR,          // 300-600cm: Low freq, slow beeps
    ALERT_MEDIUMFAR,
    ALERT_MEDIUM,       // 150-300cm: Medium freq, medium beeps  
    ALERT_CLOSE,        // 50-150cm: High freq, fast beeps
    ALERT_IMMEDIATE     // 0-50cm: Very high freq, rapid beeps
} alert_level_t;

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

static const alert_config_t alert_configs[] = {
    {0,    0,   0,    "Silent",    LedColor::GREEN,  1, 100, 0.1}, // ALERT_SILENT
    {800,  50,  50, "Far",         LedColor::GREEN,   1, 200, 0.3}, // ALERT_FAR
    {1000,  50,  50, "MediumFar",  LedColor::YELLOW,   1, 200, 0.4}, // ALERT_MEDIUMFAR
    {1200, 50,  80, "Medium",      LedColor::ORANGE, 1, 300, 0.5}, // ALERT_MEDIUM
    {1800, 50,  120,  "Close",     LedColor::RED,    1, 200, 0.7}, // ALERT_CLOSE
    {2500, 70, 200,  "Immediate",  LedColor::WHITE,  2, 100, 1}  // ALERT_IMMEDIATE
};

typedef struct {
    alert_level_t current_level;
    bool is_beeping;
    uint32_t beep_start_time;
    uint32_t last_beep_time;
    uint32_t beep_count;
} beeper_state_t;

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
   
    } else {
        return ALERT_SILENT;
    }
}

// Update beeper based on current alert level
//void update_beeper_alerts(alert_level_t new_level) {
    void update_beeper_alerts(alert_level_t new_level) {

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check if alert level changed
    if (new_level != beeper.current_level) {
        beeper.current_level = new_level;
        beeper.last_beep_time = 0;  // Reset timing to start new pattern immediately
        beeper.is_beeping = false;
        set_beeper_tone(0, false);  // Stop current beep

  
 //       ESP_LOGI(TAG, "🔊 Alert level changed to: %s", alert_configs[new_level].description);


} 
    
   const alert_config_t *config = &alert_configs[beeper.current_level];

         // Blink LED
        rgb_led_blink(config->led_color, config->led_blink_times, config->led_blink_delay_ms, config->led_strength_val);
        ESP_LOGI(TAG, "LED Color at update_beeper_alerts: %s (%d)", 
        led_color_to_string(config->led_color),
        (int)config->led_color);


    // Handle silent mode
    if (beeper.current_level == ALERT_SILENT) {
        if (beeper.is_beeping) {
            set_beeper_tone(0, false);
            beeper.is_beeping = false;
            rgb_led_set_color(0,0,0); // turn LED off
        }
        return;
    }

      
    // Handle beeping pattern
    if (beeper.is_beeping) {
        // Check if current beep should end
        if (current_time - beeper.beep_start_time >= config->beep_duration_ms) {    
            set_beeper_tone(0, false);
            beeper.is_beeping = false;
            beeper.last_beep_time = current_time;
            beeper.beep_count++;
        }
    } else {
        // Check if it's time for next beep
        if (current_time - beeper.last_beep_time >= config->beep_interval_ms) {
            set_beeper_tone(config->frequency_hz, true);
            beeper.is_beeping = true;
            beeper.beep_start_time = current_time;
                // Call rgb_led_blink with parameters from config



        }
    }
}


extern "C" void app_main(void)
{
 uint32_t last_sensor_read = 0;
//Define the i2c bus configuration
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
/*
   i2c_master_bus_config_t i2c_mst_config = {

            .i2c_port = i2c_port,
            .sda_io_num = 1,
            .scl_io_num = 2,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 5,
            .flags.enable_internal_pullup = true,
            .flags.allow_pd = true,
    };*/

    i2c_master_bus_handle_t bus_handle;

   // esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t *bus_config, i2c_master_bus_handle_t *ret_bus_handle);


    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    //Define the i2c device configuration
    i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = VL53L5CX_DEFAULT_I2C_ADDRESS >> 1,
            .scl_speed_hz = VL53L5CX_MAX_CLK_SPEED,
    };

    /*********************************/
    /*   VL53L5CX ranging variables  */
    /*********************************/

    uint8_t 				status, loop, isAlive, isReady, i;
    VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
    VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */


    /*********************************/
    /*      Customer platform        */
    /*********************************/

    Dev.platform.bus_config = i2c_mst_config;

    //Register the device
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &Dev.platform.handle);

    /* (Optional) Reset sensor */
    Dev.platform.reset_gpio = GPIO_NUM_5;
    VL53L5CX_Reset_Sensor(&(Dev.platform));

    /* (Optional) Set a new I2C address if the wanted address is different
    * from the default one (filled with 0x20 for this example).
    */
    //status = vl53l5cx_set_i2c_address(&Dev, 0x20);


    /*********************************/
    /*   Power on sensor and init    */
    /*********************************/

    /* (Optional) Check if there is a VL53L5CX sensor connected */
    status = vl53l5cx_is_alive(&Dev, &isAlive);
    if(!isAlive || status)
    {
        printf("VL53L5CX not detected at requested address\n");
        return;
    }

    /* (Mandatory) Init VL53L5CX sensor */
    status = vl53l5cx_init(&Dev);
    if(status)
    {
        printf("VL53L5CX ULD Loading failed\n");
        return;
    }

    printf("VL53L5CX ULD ready ! (Version : %s)\n",
           VL53L5CX_API_REVISION);

  const uint32_t SENSOR_READ_INTERVAL = 1; // 100ms = 10Hz
    
    /*********************************/
    /*	Set nb target per zone       */
    /*********************************/

    /* Each zone can output between 1 and 4 targets. By default the output
     * is set to 1 targets, but user can change it using macro
     * VL53L5CX_NB_TARGET_PER_ZONE located in file 'platform.h'.
     */

    // Initialize piezo beeper
    esp_err_t ret = init_piezo_beeper();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize piezo beeper");
       //   rgb_led_blink(LedColor::RED, 1, 50, 255);
        return;
    }
  
  for (int i = 0; i < 3; i++) {
        set_beeper_tone(1000, true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_beeper_tone(0, false);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
 rgb_led_init();
 rgb_led_blink(LedColor::RED, 1, 50, 255);
    /*********************************/
    /*         Ranging loop          */
    /*********************************/


    // Add this array to map your flat sensor indices to grid positions
const uint8_t gridLayout[4][4] = {
    {0, 1, 2, 3},  // Bottom row (index 0-3)
    {4, 5, 6, 7},  // Middle row (index 4-7)
    {8, 9, 10, 11}, // Upper middle row (index 8-11)
    {12, 13, 14, 15} // Top row (index 12-15)
};
// Array to store alert levels for each grid position
alert_level_t gridAlerts[16];

uint16_t primary_distance_total = 0;

    status = vl53l5cx_start_ranging(&Dev);

    loop = 0;
    while(1) //loop < 1000
    {

              uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        /* Use polling function to know when a new measurement is ready.
         * Another way can be to wait for HW interrupt raised on PIN A1
         * (INT) when a new measurement is ready */
      if (current_time - last_sensor_read >= SENSOR_READ_INTERVAL) {
            last_sensor_read = current_time;
 // rgb_led_blink(LedColor::BLUE, 1, 50, 100);
 // rgb_led_blink(LedColor::RED, 1, 50, 100);

        status = vl53l5cx_check_data_ready(&Dev, &isReady);

        if(isReady)
        {
            vl53l5cx_get_ranging_data(&Dev, &Results);

            /* As the sensor is set in 4x4 mode by default, we have a total
             * of 16 zones to print */
 //           printf("Print data no : %3u\n", Dev.streamcount);

primary_distance_total = 0;

            for(i = 0; i < 16; i++)
            {
               // vTaskDelay(pdMS_TO_TICKS(500));
                /* Print per zone results. These results are the same for all targets */
/*                printf("Zone %3u : nb_target_detected:%2u, ambient_per_spad:%6lu, nb_spads_enabled:%6lu, ",
                       i,
                       Results.nb_target_detected[i],
                       Results.ambient_per_spad[i],
                       Results.nb_spads_enabled[i]);
*/
                for(int j = 0; j < VL53L5CX_NB_TARGET_PER_ZONE; j++)
                
                {
                    /* Print per target results. These results depends of the target nb */
                    uint16_t idx = VL53L5CX_NB_TARGET_PER_ZONE * i + j;
 /*                  printf("Target[%1u] : target_status:%2u, %4d mm, signal_per_spad:%6lu, range_sigma_mm:%3u, \n",
                           j,
                           Results.target_status[idx],
                           Results.distance_mm[idx],
                           Results.signal_per_spad[idx],
                           Results.range_sigma_mm[idx]);
*/

uint16_t primary_distance = Results.distance_mm[idx]/10;
gridAlerts[i] = get_alert_level(primary_distance);
primary_distance_total += primary_distance;


// Print vertical and horizontal position if needed
if (i == 0) {
 //   printf("Grid initialized:\n");
}
/*
                        if ( i < 4 ){ //Results.distance_mm[idx] < 3000 &&
                            
                             alert_level_t alert_level = get_alert_level(primary_distance);
                             update_beeper_alerts(alert_level);
                            printf("BOTTOM : [%1u] : %4d mm -> %d\n", j, Results.distance_mm[idx], alert_level);
                       }   
                       if ( i < 7 && i > 3 ){ //Results.distance_mm[idx] < 3000 &&
                            uint16_t primary_distance = Results.distance_mm[idx]/10;
                             alert_level_t alert_level = get_alert_level(primary_distance);
                             update_beeper_alerts(alert_level);
                            printf("ALMOST BOTTOM : [%1u] : %4d mm -> %d\n", j, Results.distance_mm[idx], alert_level);
                       }   
                       if ( i < 12 && i > 7 ){ //Results.distance_mm[idx] < 3000 &&
                            uint16_t primary_distance = Results.distance_mm[idx]/10;
                             alert_level_t alert_level = get_alert_level(primary_distance);
                             update_beeper_alerts(alert_level);
                            printf("ALMOST TOP : [%1u] : %4d mm -> %d\n", j, Results.distance_mm[idx], alert_level);
                       }   

                       if ( i < 16 && i > 11 ){ //Results.distance_mm[idx] < 3000 &&
                            uint16_t primary_distance = Results.distance_mm[idx]/10;
                             alert_level_t alert_level = get_alert_level(primary_distance);
                             update_beeper_alerts(alert_level);
                            printf("TOP : [%1u] : %4d mm -> %d\n", j, Results.distance_mm[idx], alert_level);
                       }

*/
/*
                            if (i == 5){
                            uint16_t primary_distance_x = Results.distance_mm[idx]/10;
                             alert_level_t alert_level = get_alert_level(primary_distance_x);
                             update_beeper_alerts(alert_level);

                            }
*/
                       
                }
                //printf("\n");


            } // all grid squares have been read

int average_level = primary_distance_total/16;
 alert_level_t alert_level_average = get_alert_level(average_level);
                             update_beeper_alerts(alert_level_average);

       //     printf("Grid (alert level): ");
for (int row = 0; row <= 3; row++) {  // Start from top row to bottom
    for (int col = 3; col >= 0; col--) {
        // Get the alert level for this grid position
        alert_level_t level = gridAlerts[gridLayout[row][col]];
       
        // Print the alert level with spaces and pipes for visual clarity
        switch(level) {
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

/* 
         // Visual representation (using different characters)
        switch(level) {
            case alert_level_t::ALERT_SILENT: {
                putchar(' ');  // Space for silent
                break;
            }
            case alert_level_t::ALERT_FAR: {
                putchar('░');  // Low density block
                break;
            }
            case alert_level_t::ALERT_MEDIUMFAR: {
                putchar('▒');  // Medium density block
                break;
            }

            case alert_level_t::ALERT_MEDIUM: {
                putchar('▒');  // Medium density block
                break;
            }
            case alert_level_t::ALERT_CLOSE: {
                putchar('▓');  // High density block
                break;
            }
            case alert_level_t::ALERT_IMMEDIATE: {
                putchar('█');  // Solid block
                break;
            }
        }
 */
        // Add vertical separator after each row except last in row
        if (col < 3) {
            printf(" | ");
        }
        
        // Add horizontal separator after each row except bottom row
        if (col == 3) {
            printf("\n");
        }
    }

}

 //           printf("\n");
            loop++;
        }

    } //  const uint32_t SENSOR_READ_INTERVAL = 100; // 100ms = 10Hz
    
        /* Wait a few ms to avoid too high polling (function in platform
         * file, not in API) */
        VL53L5CX_WaitMs(&(Dev.platform), 5);
    }

    status = vl53l5cx_stop_ranging(&Dev);
    set_beeper_tone(0, false);  // Stop current beep
  //  printf("End of ULD demo\n");
}
