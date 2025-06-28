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
 #define WATCHDOG_TIMEOUT_MSEC 20  // Reset if no feed within 10 milliseconds

/// #include "blink_task.h"
 
////// LED
 #include "led_strip.h"
 #include "led_strip_rmt.h"
 #include "math.h"

 #include "driver/gpio.h"

 //#include "vl53l1x.h"
//#include "hcsr04_driver.h"
#include <ultrasonic.h>

#define TRIGGER_GPIO GPIO_NUM_17
#define ECHO_GPIO GPIO_NUM_16
#define MAX_DISTANCE_CM 400 // 5m max

#define TRIGGER_GPIO_2 GPIO_NUM_3
#define ECHO_GPIO_2 GPIO_NUM_8

#include <gps.h>

#include "nmea_example.h"
#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"
#include "gpvtg.h"
#include "gptxt.h"
#include "gpgsv.h"

#include "driver/uart.h"

 #ifdef useVibrator

 
 // vibration
 //#include "vibramotor.h"
//#include <driver/pwm.h>
#define LEDC_GPIO 39
//#define  LEDC_OUTPUT_IO 39
#define SAMPLE_CNT 32


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (39) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (5) // Frequency in Hertz. Set frequency at 4 kHz


static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
   //     .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
//       .intr_type      = LEDC_INTR_DISABLE,

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
         .gpio_num       = LEDC_OUTPUT_IO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
  
       
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
#endif

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
#define BLUE_PIN_1  GPIO_NUM_14
#define GREEN_PIN_1  GPIO_NUM_37

// Define GPIO pin number
//#define BLINK_PIN GPIO_NUM_35

#define centreAlert_WarnLevel 3

#define acc_Move_limit 10
static float last_pitch;
static float last_roll;
static bool vl53l5cx_running = false;
static TaskHandle_t xHandle_mpu;
static TaskHandle_t xHandle_mpu_processor;
static TaskHandle_t xHandle_ultrasonic;
static TaskHandle_t xHandle_ultrasonic_2;
static TaskHandle_t xHandle_vl53l5cx;
static TaskHandle_t xHandle_vibrator;
static TaskHandle_t xHandle_blink;
static TaskHandle_t xHandle_GPS;

#define movement_timeout_msec 30
static int last_movement = 0;
// Define Notification Bits to act as commands
#define START_BIT   (1 << 0) // Bit 0 for START command
#define STOP_BIT    (1 << 1) // Bit 1 for STOP command

bool vl53l5cx_hasMutex = false;
bool mpu_hasMutex = false;

// #define WS2812_GPIO         48
// #define WS2812_LED_COUNT     1
 
static const char *TAG = "WALKING_AID";

//static led_strip_handle_t led_strip = NULL;
// Piezo beeper pin
#define PIEZO_BEEPER_PIN        GPIO_NUM_38   // PWM pin for piezo beeper
#define VIBRATORPIN GPIO_NUM_10
#define VIBRATORPIN2 GPIO_NUM_11

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
#define  useAccelerometer 1
//#undef useAccelerometer
#undef useRGBLed
#undef useVibrator
#define useUltrasound 
#undef useVl53l5cx
#undef useUartA02YYUW
#define useGPS
#undef useGPS_2

#undef  useUart 

#define useVibrator2
/*
// vl53l1x ///
static VL53L1_Dev_t dev;
#define VLTAG "VL53L1X"
VL53L1_Error status = VL53L1_ERROR_NONE;
VL53L1_RangingMeasurementData_t rangingData;
uint8_t dataReady = 0;
uint16_t range;
*/

/// UART
//#define RXD_PIN 16
//#define TXD_PIN 17
#define UART_NUM UART_NUM_1
#define UART_BUFFER_SIZE (1024)
#define UART_TIMEOUT_MS 20

//#ifdef useGPS
#define UART_GPS UART_NUM_2 
#define UART_GPS_TXD GPIO_NUM_6 
#define UART_GPS_RXD GPIO_NUM_7 
//#endif


 int ALERT_IMMEDIATE_LIMIT = 40;
  int ALERT_CLOSE_LIMIT  = 60;
  int ALERT_MEDIUM_LIMIT  = 100;
   int ALERT_MEDIUMFAR_LIMIT  = 150;
  int ALERT_FAR_LIMIT  = 200;
    int ALERT_VERYFAR_LIMIT  = 250;

 
static uint16_t average_cm = 150;
static uint16_t  total_average_cm = 150;

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
SemaphoreHandle_t uart_mutex;
SemaphoreHandle_t  ultrasound_mutex;
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
alert_level_t get_alert_level(uint16_t distance_cm, uint16_t measured_average_cm);
void update_beeper_alerts(alert_level_t new_level, int direction);

// Task functions
extern "C" void read_and_parse_nmea();
//extern "C" void nmea_example_init_interface(void);

#ifdef useRGBLed
void led_control_task(void *pvParameters);
#endif
#ifdef useAccelerometer
void mpu9250_reader_task(void *pvParameters);
void data_processor_task(void *pvParameters);
#endif


void vl53l5cx_reader_task(void *pvParameters);
bool vl53l5cx_recover(VL53L5CX_Configuration *dev);

//void blink_task(void *pvParameters);
// Task prototype
//void blink_task_init(gpio_num_t pin, uint8_t times, uint8_t interval_ms);

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

#ifdef useGPS_2

#define UART_RX_BUF_SIZE 1024
#define UART_RX_PIN 17

static char s_buf[UART_RX_BUF_SIZE + 1];
static size_t s_total_bytes;
static char *s_last_buf_end;

extern "C" void nmea_example_init_interface(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM,
                                 UART_PIN_NO_CHANGE, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0));
}

void nmea_example_read_line(char **out_line_buf, size_t *out_line_len, int timeout_ms)
{
    *out_line_buf = NULL;
    *out_line_len = 0;

    if (s_last_buf_end != NULL) {
        /* Data left at the end of the buffer after the last call;
         * copy it to the beginning.
         */
        size_t len_remaining = s_total_bytes - (s_last_buf_end - s_buf);
        memmove(s_buf, s_last_buf_end, len_remaining);
        s_last_buf_end = NULL;
        s_total_bytes = len_remaining;
    }

    /* Read data from the UART */
    int read_bytes = uart_read_bytes(UART_NUM,
                                     (uint8_t *) s_buf + s_total_bytes,
                                     UART_RX_BUF_SIZE - s_total_bytes, pdMS_TO_TICKS(timeout_ms));
    if (read_bytes <= 0) {
        return;
    }
    s_total_bytes += read_bytes;

    /* find start (a dollar sign) */
    char *start = memchr(s_buf, '$', s_total_bytes);
    if (start == NULL) {
        s_total_bytes = 0;
        return;
    }

    /* find end of line */
    char *end = memchr(start, '\r', s_total_bytes - (start - s_buf));
    if (end == NULL || *(++end) != '\n') {
        return;
    }
    end++;

    end[-2] = NMEA_END_CHAR_1;
    end[-1] = NMEA_END_CHAR_2;

    *out_line_buf = start;
    *out_line_len = end - start;
    if (end < s_buf + s_total_bytes) {
        /* some data left at the end of the buffer, record its position until the next call */
        s_last_buf_end = end;
    } else {
        s_total_bytes = 0;
    }
}


extern "C" void read_and_parse_nmea()
{
    while (1) {
        char fmt_buf[32];
        nmea_s *data;

        char *start;
        size_t length;
        nmea_example_read_line(&start, &length, 100 /* ms */);
        if (length == 0) {
            continue;
        }

        /* handle data */
        data = nmea_parse(start, length, 0);
        if (data == NULL) {
            printf("Failed to parse the sentence!\n");
            printf("  Type: %.5s (%d)\n", start + 1, nmea_get_type(start));
        } else {
            if (data->errors != 0) {
                printf("WARN: The sentence struct contains parse errors!\n");
            }

            if (NMEA_GPGGA == data->type) {
                printf("GPGGA sentence\n");
                nmea_gpgga_s *gpgga = (nmea_gpgga_s *) data;
                printf("Number of satellites: %d\n", gpgga->n_satellites);
                printf("Altitude: %f %c\n", gpgga->altitude,
                       gpgga->altitude_unit);
            }

            if (NMEA_GPGLL == data->type) {
                printf("GPGLL sentence\n");
                nmea_gpgll_s *pos = (nmea_gpgll_s *) data;
                printf("Longitude:\n");
                printf("  Degrees: %d\n", pos->longitude.degrees);
                printf("  Minutes: %f\n", pos->longitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
                printf("Latitude:\n");
                printf("  Degrees: %d\n", pos->latitude.degrees);
                printf("  Minutes: %f\n", pos->latitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
                strftime(fmt_buf, sizeof(fmt_buf), "%H:%M:%S", &pos->time);
                printf("Time: %s\n", fmt_buf);
            }

            if (NMEA_GPRMC == data->type) {
                printf("GPRMC sentence\n");
                nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
                printf("Longitude:\n");
                printf("  Degrees: %d\n", pos->longitude.degrees);
                printf("  Minutes: %f\n", pos->longitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
                printf("Latitude:\n");
                printf("  Degrees: %d\n", pos->latitude.degrees);
                printf("  Minutes: %f\n", pos->latitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
                strftime(fmt_buf, sizeof(fmt_buf), "%d %b %T %Y", &pos->date_time);
                printf("Date & Time: %s\n", fmt_buf);
                printf("Speed, in Knots: %f\n", pos->gndspd_knots);
                printf("Track, in degrees: %f\n", pos->track_deg);
                printf("Magnetic Variation:\n");
                printf("  Degrees: %f\n", pos->magvar_deg);
                printf("  Cardinal: %c\n", (char) pos->magvar_cardinal);
                double adjusted_course = pos->track_deg;
                if (NMEA_CARDINAL_DIR_EAST == pos->magvar_cardinal) {
                    adjusted_course -= pos->magvar_deg;
                } else if (NMEA_CARDINAL_DIR_WEST == pos->magvar_cardinal) {
                    adjusted_course += pos->magvar_deg;
                } else {
                    printf("Invalid Magnetic Variation Direction!\n");
                }

                printf("Adjusted Track (heading): %f\n", adjusted_course);
            }

            if (NMEA_GPGSA == data->type) {
                nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *) data;

                printf("GPGSA Sentence:\n");
                printf("  Mode: %c\n", gpgsa->mode);
                printf("  Fix:  %d\n", gpgsa->fixtype);
                printf("  PDOP: %.2lf\n", gpgsa->pdop);
                printf("  HDOP: %.2lf\n", gpgsa->hdop);
                printf("  VDOP: %.2lf\n", gpgsa->vdop);
            }

            if (NMEA_GPGSV == data->type) {
                nmea_gpgsv_s *gpgsv = (nmea_gpgsv_s *) data;

                printf("GPGSV Sentence:\n");
                printf("  Num: %d\n", gpgsv->sentences);
                printf("  ID:  %d\n", gpgsv->sentence_number);
                printf("  SV:  %d\n", gpgsv->satellites);
                printf("  #1:  %d %d %d %d\n", gpgsv->sat[0].prn, gpgsv->sat[0].elevation, gpgsv->sat[0].azimuth, gpgsv->sat[0].snr);
                printf("  #2:  %d %d %d %d\n", gpgsv->sat[1].prn, gpgsv->sat[1].elevation, gpgsv->sat[1].azimuth, gpgsv->sat[1].snr);
                printf("  #3:  %d %d %d %d\n", gpgsv->sat[2].prn, gpgsv->sat[2].elevation, gpgsv->sat[2].azimuth, gpgsv->sat[2].snr);
                printf("  #4:  %d %d %d %d\n", gpgsv->sat[3].prn, gpgsv->sat[3].elevation, gpgsv->sat[3].azimuth, gpgsv->sat[3].snr);
            }

            if (NMEA_GPTXT == data->type) {
                nmea_gptxt_s *gptxt = (nmea_gptxt_s *) data;

                printf("GPTXT Sentence:\n");
                printf("  ID: %d %d %d\n", gptxt->id_00, gptxt->id_01, gptxt->id_02);
                printf("  %s\n", gptxt->text);
            }

            if (NMEA_GPVTG == data->type) {
                nmea_gpvtg_s *gpvtg = (nmea_gpvtg_s *) data;

                printf("GPVTG Sentence:\n");
                printf("  Track [deg]:   %.2lf\n", gpvtg->track_deg);
                printf("  Speed [kmph]:  %.2lf\n", gpvtg->gndspd_kmph);
                printf("  Speed [knots]: %.2lf\n", gpvtg->gndspd_knots);
            }

            nmea_free(data);
        }

vTaskDelay(pdMS_TO_TICKS(1000));

    }
}
#endif


#ifdef useGPS
/*
typedef struct
{
    double latitude;
    double longitude;
    double speed_kmh; // 单位：千米每小时
    double speed_ms;  // 单位：米每秒
} GPS_data;
*/


void gps_task(void *arg)
{


for (;;) {

            uint32_t notification_value = 0;

        // 1. Wait indefinitely for any notification (e.g., the START signal)
        // This call will block until xTaskNotify is called on this task.
        xTaskNotifyWait(0,           /* Don't clear any bits on entry */
                        ULONG_MAX,   /* Clear all bits on exit */
                        &notification_value, /* Stores the notification value */
                        portMAX_DELAY);      /* Block forever */

        // 2. Check if the START signal was received
        if (notification_value & START_BIT) {
            ESP_LOGI(TAG, "START signal for GPS reading loop.");


ESP_LOGI("GPS", "GPS starting");

ESP_LOGD("GPS", "GPS Waiting for uart_mutex ...");
if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {

//_hasMutex = true;
vTaskDelay(pdMS_TO_TICKS(100));


   // const char *TAG = "GPS";
    while (1)
    {
        GPS_data gps_data = gps_get_value();
        ESP_LOGI("GPS", "lat:%f, lon:%f alt:%f", gps_data.latitude, gps_data.longitude, gps_data.altitude); 
        ESP_LOGI("GPS", "speed:%f", gps_data.speed_ms); 
        ESP_LOGI("GPS", "course:%f", gps_data.course);  
        ESP_LOGI("GPS", "Date:%d/%d/%d Time:%d:%d:%d",gps_data.day, gps_data.month, gps_data.year, gps_data.hour, gps_data.minute,gps_data.second); 
// Date:25/6/28 Time:15:34:42
       // ESP_LOGI(TAG, "======================================================\n");
         vTaskDelay(2000);
       // vTaskDelay(1000 / portTICK_RATE_MS);

                      // --- CRITICAL PART: Check for a STOP signal ---
                uint32_t stop_signal_value = 0;
                // Use a zero timeout to check instantly without blocking
                xTaskNotifyWait(0, ULONG_MAX, &stop_signal_value, 0);

                // 3. If a STOP signal was sent, break the inner loop
                if (stop_signal_value & STOP_BIT) {
                    ESP_LOGI(TAG, "STOP signal for GPS loop.");
                    break; // Exit the inner "running" loop
                }

    }

    xSemaphoreGive(uart_mutex);
ESP_LOGD("GPS", "GPS Giving uart_mutex ...");
}  // got mutex

} // startbit
} // outer loop wait for startbit

}
#endif

// Task parameters structure (for potential future expansion)
typedef struct {
    gpio_num_t pin;
    uint8_t times;
    uint32_t interval_ms;
} blink_task_params_t;

// Static storage for task parameters
static blink_task_params_t blink_task_params;

// **Embedded Blink Task Function**
static void blink_task(void *pvParameter) {
    // Configure GPIO pin as output
 //   gpio_pad_select_gpio(BLINK_PIN);
    gpio_set_direction(blink_task_params.pin, GPIO_MODE_OUTPUT);

//ESP_LOGI("BLINK", "blink_task %d %d %d", blink_task_params.pin,blink_task_params.times, blink_task_params.interval_ms );


    // Blink 'times' times with 'interval_ms' between blinks
    for (uint8_t i = 0; i < blink_task_params.times; i++) {
        gpio_set_level(blink_task_params.pin, 1);
        vTaskDelay(pdMS_TO_TICKS(blink_task_params.interval_ms / 2));
        gpio_set_level(blink_task_params.pin, 0);
        vTaskDelay(pdMS_TO_TICKS(blink_task_params.interval_ms - (blink_task_params.interval_ms / 2)));
    }
    gpio_set_level(blink_task_params.pin, 0); // Final state
    vTaskDelete(NULL);
}

// **Embedded Function to Initialize Blink Task**
static void blink_task_init( gpio_num_t pin, uint8_t times, uint32_t interval_ms) {
    blink_task_params.pin = pin;
    blink_task_params.times = times;
    blink_task_params.interval_ms = interval_ms;
TaskHandle_t xHandle_blink = NULL;
//  vTaskDelete( xHandle_blink );

 //     if (xHandle_blink = NULL){
//ESP_LOGI("BLINK", "blink_task_init %d %d %d", pin,times, interval_ms );

    xTaskCreate(blink_task, "BlinkTask", 1024, NULL, 2, &xHandle_blink);
 //     }

}

///// vibrator

typedef struct {
    gpio_num_t pin;
    uint8_t times;
    uint32_t interval_ms;
} vibrator_task_params_t;

// Static storage for task parameters
static vibrator_task_params_t vibrator_task_params;

// **Embedded Blink Task Function**
static void vibrator_task(void *pvParameter) {
    // Configure GPIO pin as output
 //   gpio_pad_select_gpio(BLINK_PIN);
    gpio_set_direction(vibrator_task_params.pin, GPIO_MODE_OUTPUT);

//ESP_LOGI("BLINK", "blink_task %d %d %d", blink_task_params.pin,blink_task_params.times, blink_task_params.interval_ms );
//ESP_LOGI("VIBRA", "vibrator_task %d %d %d", vibrator_task_params.pin,vibrator_task_params.times, vibrator_task_params.interval_ms );


    // Blink 'times' times with 'interval_ms' between blinks
    for (uint8_t i = 0; i < vibrator_task_params.times; i++) {
        gpio_set_level(vibrator_task_params.pin, 1);
        vTaskDelay(pdMS_TO_TICKS(vibrator_task_params.interval_ms / 2));
        gpio_set_level(vibrator_task_params.pin, 0);
        vTaskDelay(pdMS_TO_TICKS(vibrator_task_params.interval_ms - (vibrator_task_params.interval_ms / 2)));
    }
    gpio_set_level(vibrator_task_params.pin, 0); // Final state
    vTaskDelete(NULL);
}

static void vibrator_task_init( gpio_num_t pin, uint8_t times, uint32_t interval_ms2) {
    vibrator_task_params.pin = pin;
    vibrator_task_params.times = times;
    vibrator_task_params.interval_ms = interval_ms2;

//TaskHandle_t xHandle_vibrator = NULL;
// vTaskDelete( xHandle_vibrator );

 //   if (!xHandle_vibrator){

//ESP_LOGI("VIBRA", "vibrator_task_init %d %d %d", pin,times, interval_ms2 );

    xTaskCreate(vibrator_task, "vibrator_task", 1024, NULL, 2, &xHandle_vibrator);
 //   }

}



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
   vTaskDelay(pdMS_TO_TICKS(100));
static int mpu9250_update_period = 100;
    while (1)
    {
         vTaskDelay(pdMS_TO_TICKS(200));
  
        uint32_t current_time = xTaskGetTickCount();
        float dt = (float)(current_time - last_update_time) / (float)configTICK_RATE_HZ;
        
        last_update_time = current_time;
 //   ESP_LOGI("MPU9250_TASK", "mpu9250 check update cur %d last %d",  current_time, last_update_time );

 //  last_update_time = 0;
//if (current_time - last_update_time > mpu9250_update_period ) {
   //   last_update_time = current_time;
  
 ESP_LOGD("MUTEX2", "mpu9250 Waiting for g_i2c_bus mutex...");
 
        ESP_LOGD("MPU9250_TASK", "mpu9250 updating");

           if (xSemaphoreTake(g_i2c_bus_mutex, portMAX_DELAY) == pdTRUE) {

mpu_hasMutex = true;


  ESP_LOGD("MUTEX2", "mpu9250 g_i2c_bus Mutex taken.");

  ESP_LOGD("MPU9250_TASK", "Doing mi2c_master_transmit_receive");


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

              ESP_LOGD("MPU9250_TASK", "mi2c_master_transmit_receive success");

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
 ESP_LOGD("MPU9250_TASK", "total_accel_magnitude: %f", total_accel_magnitude);
// When the device is still, magnitude should be ~1.0. During free-fall, it's ~0.0.
    if (total_accel_magnitude > 1.3) { // Threshold for detecting free-fall
        ESP_LOGI("FALL_DETECT", "FREE-FALL DETECTED! Magnitude: %.2f g", total_accel_magnitude);
 //   blink_task_init(GPIO_NUM_35, 2, 500); // 
 blink_task_init(GPIO_NUM_36, 5, 500); // 
//  blink_task_init(GPIO_NUM_37, 2, 500); // 

 // vTaskDelay(pdMS_TO_TICKS(1000));

    // You could trigger an alert here or set a flag for another task to see.
    }

               
//if (mpu_hasMutex){
            xSemaphoreGive(g_i2c_bus_mutex);
            mpu_hasMutex = false;
              ESP_LOGD("MUTEX2", "mpu9250 g_i2c_bus Mutex given.");
//}
      
    }

//} // mpu9250_update_period
  
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
 ESP_LOGD("PROCESSOR_TASK", "received data.");
/*
         straight    Pitch: -25.00°, Roll: 159.13° 
          up   Pitch: -48.69°, Roll: 124.07°
        down Pitch: -53.90°, Roll: 179.25°
        lean right: Pitch: -14.88°, Roll: 186.32°
        lean left: Pitch: -40.04°, Roll: 130.50°
turn right: Pitch: -38.04°, Roll: 161.72°
turn left: Pitch: -68.80°, Roll: 162.69°
*/


/*
if (received_data.pitch < -50 ){
    ESP_LOGI("ACCEL", "LEFT");
}

if (received_data.pitch < -40 ){
    ESP_LOGI("ACCEL", "UP");
}


if (received_data.roll > 130 ){
    ESP_LOGI("ACCEL", "lean right");
}
if (received_data.roll > 170 ){
    ESP_LOGI("ACCEL", "lean right");
}
*/

uint32_t current_time = xTaskGetTickCount();

if ( abs(received_data.pitch -  last_pitch ) > acc_Move_limit || abs(received_data.roll -  last_roll) > acc_Move_limit ) {

 ESP_LOGI("PROCESSOR_TASK", "Device is moved %.1f  %.1f current_time: %d last_movement: %d", abs(received_data.pitch -  last_pitch ), abs(received_data.roll -  last_roll), current_time, last_movement   );
// PROCESSOR_TASK: Device is moved (Roll: -21.1). (Pitch: 5.0)  26.4  8.1 time: 7089
// PROCESSOR_TASK: Device is moved (Roll: -26.9). (Pitch: 15.3)  25.3  13.9 current_time: 1374 last_movement: 1333
// PROCESSOR_TASK: Device is moved (Roll: -14.0). (Pitch: 93.5)  1.0  32.7 current_time: 9440 last_movement: 1374
 //   uint32_t current_time = xTaskGetTickCount();
   // 35.4  7.0  current_time: 27785 last_movement: 10777


   //     float dt = (float)(current_time - last_update_time) / (float)configTICK_RATE_HZ;
    
 //if (( current_time - last_movement ) > movement_timeout_sec * 1000){


// if (!vl53l5cx_running ){
// start vl53l5cx task
        // 3. Give the semaphore. This will unblock the worker_task.
 //       ESP_LOGW("PROCESSOR_TASK", "Signaling the worker task to start!");
  //      xSemaphoreGive(g_binary_semaphore);
#ifdef useVl53l5cx
    ESP_LOGD("PROCESSOR_TASK", "Sending START signal to xHandle_vl53l5cx.");
     xTaskNotify(xHandle_vl53l5cx, START_BIT, eSetBits);
#endif

#ifdef useUltrasound
 ESP_LOGI("PROCESSOR_TASK", "Sending START signal to xHandle_ultrasonic.");
  xTaskNotify( xHandle_ultrasonic, START_BIT, eSetBits);
#endif
  #ifdef useUltrasound_2
   ESP_LOGI("PROCESSOR_TASK", "Sending START signal to xHandle_ultrasonic 2.");
  xTaskNotify( xHandle_ultrasonic_2, START_BIT, eSetBits);
#endif

#ifdef useGPS
ESP_LOGD("PROCESSOR_TASK", "Sending START signal to xHandle_GPS.");
xTaskNotify( xHandle_GPS, START_BIT, eSetBits);
#endif

// 1. Declare a handle for the worker task
//TaskHandle_t g_worker_task_handle = NULL;

// xTaskNotifyGive(g_worker_task_handle);
// ESP_LOGI("PROCESSOR_TASK", "vTaskResume( xHandle_vl53l5cx )");
//vTaskResume( xHandle_vl53l5cx );
//vl53l5cx_running = true;
//}

 last_movement = current_time;

} // > acc_Move_limit



if (( current_time - last_movement ) > movement_timeout_msec * 1000){ //vl53l5cx_running && 
// stop vl53l5cx task
// ESP_LOGI("PROCESSOR_TASK", "vTaskSuspend( xHandle_vl53l5cx )");
 ESP_LOGD("PROCESSOR_TASK", "no movement, current_time %d last_movement %d", current_time, last_movement);

gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 
gpio_set_level(RED_PIN, 0); 
gpio_set_level(BLUE_PIN, 0);  
gpio_set_level(GREEN_PIN, 0); 

 //xSemaphoreGive(g_i2c_bus_mutex);
//ESP_LOGD("MUTEX1", "Given g_i2c_bus mutex before vTaskSuspend");
#ifdef useVl53l5cx
 ESP_LOGD("PROCESSOR_TASK", "Sending STOP signal to worker.");
 xTaskNotify(xHandle_vl53l5cx, STOP_BIT, eSetBits);
 #endif

 #ifdef useUltrasound
  ESP_LOGI("PROCESSOR_TASK", "Sending STOP_BIT  to xHandle_ultrasonic and xHandle_ultrasonic_2.");
    xTaskNotify( xHandle_ultrasonic, STOP_BIT, eSetBits);
#endif

#ifdef useUltrasound_2
    xTaskNotify( xHandle_ultrasonic_2, STOP_BIT, eSetBits);
#endif

#ifdef useGPS
ESP_LOGD("PROCESSOR_TASK", "Sending STOP signal to xHandle_GPS.");
xTaskNotify( xHandle_GPS, STOP_BIT, eSetBits);
#endif

//vTaskSuspend( xHandle_vl53l5cx );
//vl53l5cx_running = false;
     
} // 


// ESP_LOGI("VL53_TASK", "Device is moved (Roll: %.1f). (Pitch: %.1f)  %.1f  %.1f", current_orientation.pitch ,current_orientation.roll, abs(current_orientation.pitch -  last_pitch ), abs(current_orientation.roll -  last_roll)  );

last_pitch = received_data.pitch;
last_roll = received_data.roll;
/*
            if (current_orientation.pitch > 50.0) {
                 ESP_LOGI("VL53_TASK", "Device is pointed RIGHT (Pitch: %.1f). Adjusting logic.", current_orientation.pitch);
                 // e.g., Treat the top row of the sensor as the "forward" direction
            } else if (current_orientation.pitch < -40.0) {
                 ESP_LOGI("VL53_TASK", "Device is pointed LEFT (Pitch: %.1f). Ignoring floor.", current_orientation.pitch);
                 // e.g., Filter out readings from the bottom rows
            } else {
                 // Device is relatively level, use default logic
            }


            if (current_orientation.roll > 140) {
                ESP_LOGI("VL53_TASK", "Device is pointed DOWN (Pitch: %.1f). Adjusting logic.", current_orientation.pitch);
                 // e.g., Treat the top row of the sensor as the "forward" direction
            } else if (current_orientation.roll < 100) {
                ESP_LOGI("VL53_TASK", "Device is pointed UP (Pitch: %.1f). Ignoring floor.", current_orientation.pitch);
                 // e.g., Filter out readings from the bottom rows
            } else {
                 // Device is relatively level, use default logic
            }
*/





            // --- FUSED ORIENTATION DATA (Most Important Output) ---
           ESP_LOGD("PROCESSOR_TASK", "Orientation -> Pitch: %.2f°, Roll: %.2f°", received_data.pitch, received_data.roll);
        }
 vTaskDelay(pdMS_TO_TICKS(300));


    } // while 1
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

#ifdef useUart 
void init_uart(void) {
    // Configure UART parameters
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Apply UART configuration
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver with RX buffer and a small TX buffer
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUFFER_SIZE, 512, 0, NULL, 0));

    // Set UART mode explicitly
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM, UART_MODE_UART));
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
alert_level_t get_alert_level(uint16_t distance_cm, uint16_t measured_average_cm) {

    // change levels if theres nothing near
uint16_t average_cm_far = 250;
uint16_t average_cm_close = 100;
uint16_t average_cm_veryclose = 50;
/*
 if (measured_average_cm > average_cm_far){
ALERT_IMMEDIATE_LIMIT = 100;
ALERT_CLOSE_LIMIT  = 140;
ALERT_MEDIUM_LIMIT  = 180;
ALERT_MEDIUMFAR_LIMIT  = 220;
ALERT_FAR_LIMIT  = 250;
ALERT_VERYFAR_LIMIT  = 300;
    }

 if (measured_average_cm < average_cm_close){
ALERT_IMMEDIATE_LIMIT = 20;
ALERT_CLOSE_LIMIT  = 40;
ALERT_MEDIUM_LIMIT  = 60;
ALERT_MEDIUMFAR_LIMIT  = 80;
ALERT_FAR_LIMIT  = 100;
ALERT_VERYFAR_LIMIT  = 120;
    }
 if (measured_average_cm < average_cm_veryclose){
ALERT_IMMEDIATE_LIMIT = 10;
ALERT_CLOSE_LIMIT  = 20;
ALERT_MEDIUM_LIMIT  = 30;
ALERT_MEDIUMFAR_LIMIT  = 40;
ALERT_FAR_LIMIT  = 50;
ALERT_VERYFAR_LIMIT  = 60;
    }
*/

/*
ALERT_IMMEDIATE_LIMIT = 30; // measured_average_cm / 8
ALERT_CLOSE_LIMIT  = measured_average_cm / 6;
ALERT_MEDIUM_LIMIT  = measured_average_cm;
ALERT_MEDIUMFAR_LIMIT  = measured_average_cm * 1.25;
ALERT_FAR_LIMIT  = measured_average_cm * 2;
ALERT_VERYFAR_LIMIT  = measured_average_cm * 3;
*/

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

#ifdef useUart 
void uart_task(void *pvParameters) {
    uint8_t data[4];


    if (xSemaphoreTake(uart_mutex, portMAX_DELAY) == pdTRUE) {


    while (1) {
        // Read 4 bytes from UART
        int len = uart_read_bytes(UART_NUM, data, 4, pdMS_TO_TICKS(UART_TIMEOUT_MS));

        if (len == 4 && data[0] == 0xFF) {
            // Calculate checksum
            int sum = (data[0] + data[1] + data[2]) & 0xFF;
            if (sum == data[3]) {
                // Convert distance to centimeters
                int distance = (data[1] << 8) | data[2];

                if (distance > 30) {
                    ESP_LOGI(TAG, "Distance: %.2f cm", distance / 10.0);
                } else {
                    ESP_LOGW(TAG, "Below the lower limit");
                }
            } else {
                ESP_LOGE(TAG, "Checksum error (Received: 0x%02X, Expected: 0x%02X)", data[3], sum);
            }
        } else if (len > 0) {
            ESP_LOGW(TAG, "Invalid data received");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

  xSemaphoreGive(uart_mutex);

} // mutex
}
#endif

void resetPins (){
gpio_set_level(RED_PIN, 0); 
gpio_set_level(BLUE_PIN, 0);  
gpio_set_level(GREEN_PIN, 0); 
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 

}


// simplified update_beeper_alerts function
void update_beeper_alerts(alert_level_t new_level, int direction) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    const alert_config_t *config = &alert_configs[beeper.current_level];

if (direction == 1){

resetPins();


// special for immediate close, always react
if (new_level ==6){
blink_task_init(RED_PIN_1, 5, 500); // 

#ifdef useVibrator2
vibrator_task_init( VIBRATORPIN, 1, 500);

#endif
//gpio_set_level(VIBRATORPIN, 1); 

//gpio_set_level(RED_PIN_1, 1); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 
vTaskDelay(pdMS_TO_TICKS(100));
        } //new_level ==6


    // Check if alert level changed
    if (new_level != beeper.current_level) {
        beeper.current_level = new_level;
        beeper.last_beep_time = 0;
        beeper.is_beeping = false;
        set_beeper_tone(0, false);
     

    if (beeper.current_level == ALERT_SILENT) {
        if (beeper.is_beeping) {
            set_beeper_tone(0, false);
            beeper.is_beeping = false;
        }




        return;
    } 
    


 ESP_LOGI(TAG, "new_level %d", new_level);

//gpio_set_level(RED_PIN_1, 0); 
//gpio_set_level(BLUE_PIN_1, 0);  
//gpio_set_level(GREEN_PIN_1, 0); 

//gpio_set_level(VIBRATORPIN, 0); 

/*
if (new_level ==6){
    blink_task_init(RED_PIN_1, 5, 500); // 

    #ifdef useVibrator2
        vibrator_task_init( VIBRATORPIN, 20, 100);
    #endif

    //gpio_set_level(VIBRATORPIN, 1); 

/   /gpio_set_level(RED_PIN_1, 1); 
    gpio_set_level(BLUE_PIN_1, 0);  
    gpio_set_level(GREEN_PIN_1, 0); 
    vTaskDelay(pdMS_TO_TICKS(100));
}
*/

       if (new_level ==5){

#ifdef useVibrator2
vibrator_task_init( VIBRATORPIN, 5, 100);

#endif

gpio_set_level(RED_PIN_1, 1); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 
 vTaskDelay(pdMS_TO_TICKS(80));
        }
if (new_level ==4 ){
 //   blink_task_init(BLUE_PIN_1, 5, 500); // 
#ifdef useVibrator2
vibrator_task_init( VIBRATORPIN, 4, 100);
 
#endif
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 1);  
gpio_set_level(GREEN_PIN_1, 0); 
vTaskDelay(pdMS_TO_TICKS(60));
        }
if (new_level ==3 ){
#ifdef useVibrator2
vibrator_task_init( VIBRATORPIN, 3, 100);

#endif
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 1);  
gpio_set_level(GREEN_PIN_1, 1); 
 vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (new_level ==2){
vibrator_task_init( VIBRATORPIN, 1, 100);
 vTaskDelay(pdMS_TO_TICKS(50));
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 1); 
        }
        if (new_level ==1){
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 
        }
        if (new_level ==0){
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
    vTaskDelay(pdMS_TO_TICKS(200));
} // direction 1

else { // direction 2

if (new_level == 3){
//blink_task_init(BLUE_PIN_1, 5, 500); // 
blink_task_init(GREEN_PIN, 1, 20);
}


// special for immediate close, always react
if (new_level >= 4){
//blink_task_init(BLUE_PIN_1, 5, 500); // 
//blink_task_init(GREEN_PIN, 3, 100);

#ifdef useVibrator2
//vibrator_task_init( VIBRATORPIN, 2, 100);
//vTaskDelay(pdMS_TO_TICKS(100));
//vibrator_task_init( VIBRATORPIN, 2, 100);
#endif
//gpio_set_level(VIBRATORPIN, 1); 
blink_task_init(RED_PIN, 3, 100);

#ifdef useVibrator2
vibrator_task_init( VIBRATORPIN2, 2, 100);
#endif
vTaskDelay(pdMS_TO_TICKS(200));
blink_task_init(BLUE_PIN, 3, 200);
//vTaskDelay(pdMS_TO_TICKS(200));

#ifdef useVibrator2
vibrator_task_init( VIBRATORPIN2, 2, 100);
#endif
vTaskDelay(pdMS_TO_TICKS(200));

//gpio_set_level(GREEN_PIN_1, 0); 


        } //new_level ==6
vTaskDelay(pdMS_TO_TICKS(200));
}

//resetPins();
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


bool vl53l5cx_recover(VL53L5CX_Configuration *dev) {
    ESP_LOGW(TAG, "Attempting VL53L5CX recovery...");
    
    // Take I2C mutex for recovery operations
     ESP_LOGD("MUTEX3", "Waiting for I2C Mutex for recovery.");
    if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE("MUTEX3", "Could not get I2C mutex for recovery");
        vl53l5cx_hasMutex = false;
        return false;
    }
    ESP_LOGD("MUTEX3", "I2C Mutex taken for recovery.");
vl53l5cx_hasMutex = true;

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

        if (vl53l5cx_hasMutex){
        xSemaphoreGive(g_i2c_bus_mutex);
        vl53l5cx_hasMutex = false;
         ESP_LOGD("MUTEX3", "g_i2c_bus_mutex Mutex given 3.");
        }
        return false;
    }
    
    // Step 4: Re-initialize the sensor
    ESP_LOGI(TAG, "Re-initializing sensor...");
    status = vl53l5cx_init(dev);
    if (status) {
        ESP_LOGE(TAG, "Re-initialization failed with status: %d", status);
         if (vl53l5cx_hasMutex){
        xSemaphoreGive(g_i2c_bus_mutex);
        vl53l5cx_hasMutex = false;
         ESP_LOGD("MUTEX3", "g_i2c_bus_mutex Mutex given 4.");
         }
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
         ESP_LOGD("MUTEX3", "g_i2c_bus_mutex Mutex given 6.");
        return false;
    }
    
    // Release mutex
     if (vl53l5cx_hasMutex){
    xSemaphoreGive(g_i2c_bus_mutex);
    vl53l5cx_hasMutex = false;
    ESP_LOGD("MUTEX3", "g_i2c_bus_mutex Mutex given from recover.");
     }
    ESP_LOGI(TAG, "VL53L5CX recovery successful!");
    return true;
}

#ifdef useVl53l5cx
// New task function for the sensor loop
void vl53l5cx_reader_task(void *pvParameters) {

ESP_LOGI(TAG, "vl53l5cx_reader_task started. Will wait for a signal to run.");


    uint32_t minStackBytesRemaining = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Sensor Task: Initial Stack Remaining %d bytes", minStackBytesRemaining);
    static uint32_t runCounter = 0;
   
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
    const uint32_t SENSOR_READ_INTERVAL = 600; // 1ms
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

vTaskDelay(pdMS_TO_TICKS(200));

for (;;) {

            uint32_t notification_value = 0;

        // 1. Wait indefinitely for any notification (e.g., the START signal)
        // This call will block until xTaskNotify is called on this task.
        xTaskNotifyWait(0,           /* Don't clear any bits on entry */
                        ULONG_MAX,   /* Clear all bits on exit */
                        &notification_value, /* Stores the notification value */
                        portMAX_DELAY);      /* Block forever */

        // 2. Check if the START signal was received
        if (notification_value & START_BIT) {
            ESP_LOGI(TAG, "START signal received! Entering vl53l5cx reading loop.");
runCounter = 0;


ESP_LOGD("MUTEX1", "vl53l5cx Waiting for g_i2c_bus mutex...");
if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {

vl53l5cx_hasMutex = true;
vTaskDelay(pdMS_TO_TICKS(100));


  ESP_LOGD("MUTEX1", "vl53l5cx Mutex taken.");
        status = vl53l5cx_start_ranging(Dev);

vTaskDelay(pdMS_TO_TICKS(500));

    if (status) {
        ESP_LOGE(TAG, "Failed to start ranging");
         if (vl53l5cx_hasMutex){
        xSemaphoreGive(g_i2c_bus_mutex);
        vl53l5cx_hasMutex = false;
        ESP_LOGD("MUTEX1", "Failed, vl53l5cx Giving g_i2c_bus mutex...");
         }
        vTaskDelete(NULL);
       return;
    }


    while(1) {


#ifdef useAccelerometer


       if (xQueueReceive(mpu_data_queue, &current_orientation, 0) == pdPASS) {
            // New orientation data is now stored in 'current_orientation'
        }

/*
if ( abs(current_orientation.pitch -  last_pitch ) > acc_Move_limit || abs(current_orientation.roll -  last_roll) > acc_Move_limit ) {

 ESP_LOGI("VL53_TASK", "Device is moved (Roll: %.1f). (Pitch: %.1f)  %.1f  %.1f", current_orientation.pitch ,current_orientation.roll, abs(current_orientation.pitch -  last_pitch ), abs(current_orientation.roll -  last_roll)   );
}
// ESP_LOGI("VL53_TASK", "Device is moved (Roll: %.1f). (Pitch: %.1f)  %.1f  %.1f", current_orientation.pitch ,current_orientation.roll, abs(current_orientation.pitch -  last_pitch ), abs(current_orientation.roll -  last_roll)  );

last_pitch = current_orientation.pitch;
last_roll = current_orientation.roll;
*/

/*
            if (current_orientation.pitch > 50.0) {
                 ESP_LOGI("VL53_TASK", "Device is pointed RIGHT (Pitch: %.1f). Adjusting logic.", current_orientation.pitch);
                 // e.g., Treat the top row of the sensor as the "forward" direction
            } else if (current_orientation.pitch < -40.0) {
                 ESP_LOGI("VL53_TASK", "Device is pointed LEFT (Pitch: %.1f). Ignoring floor.", current_orientation.pitch);
                 // e.g., Filter out readings from the bottom rows
            } else {
                 // Device is relatively level, use default logic
            }


            if (current_orientation.roll > 140) {
                ESP_LOGI("VL53_TASK", "Device is pointed DOWN (Pitch: %.1f). Adjusting logic.", current_orientation.pitch);
                 // e.g., Treat the top row of the sensor as the "forward" direction
            } else if (current_orientation.roll < 100) {
                ESP_LOGI("VL53_TASK", "Device is pointed UP (Pitch: %.1f). Ignoring floor.", current_orientation.pitch);
                 // e.g., Filter out readings from the bottom rows
            } else {
                 // Device is relatively level, use default logic
            }
*/


#endif


bool topAlert = false;
bool bottomAlert = false;
int centreAlert = 0;



        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (current_time - last_sensor_read >= SENSOR_READ_INTERVAL) {
            last_sensor_read = current_time;


         //   vTaskDelay(pdMS_TO_TICKS(100));
            status = vl53l5cx_check_data_ready(Dev, &isReady);

//ESP_LOGD(TAG, "Sensor task started");
ESP_LOGD(TAG, "Status: %d %d", status, isReady);


            if(isReady) {

gpio_set_level(RED_PIN, 0); 
gpio_set_level(BLUE_PIN, 0);  
gpio_set_level(GREEN_PIN, 0); 

gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 


  not_ready_count = 0; // Reset counter on success
                vTaskDelay(pdMS_TO_TICKS(100));
                vl53l5cx_get_ranging_data(Dev, &Results);

                primary_distance_total = 0;
                uint16_t distance_total = 0;


                for(i = 0; i < 16; i++) {
                    for(int j = 0; j < VL53L5CX_NB_TARGET_PER_ZONE; j++) {
                        uint16_t idx = VL53L5CX_NB_TARGET_PER_ZONE * i + j;

                        uint16_t primary_distance = 0;
                        primary_distance = Results.distance_mm[idx]/10;

// if( runCounter % 10 == 0 ) {
                        distance_total += primary_distance;
                        average_cm = distance_total/(i+1);

 //                       ESP_LOGD(TAG, "average_cm %d %d", i, average_cm);
// }
if ( abs(average_cm - total_average_cm) > 50 ){
    total_average_cm = average_cm; // update total if average_cm changed
}


                        gridAlerts[i] = get_alert_level(primary_distance, total_average_cm);

//printf("grid %d:%d \n", i, gridAlerts[i]);


if ((i % 4) == 0) {

     if( primary_distance <= ALERT_CLOSE_LIMIT   ){
        topAlert = true;
     }

    }



if ((i % 4) == 3) {
 if( primary_distance <= ALERT_CLOSE_LIMIT ){ 
bottomAlert = true;
 }
}


uint8_t remainder = i % 4;
if (remainder == 1 || remainder == 2) {
    // 'i' is in gridCentre.
     primary_distance_total += primary_distance;

if(primary_distance <= ALERT_CLOSE_LIMIT){
    ESP_LOGI(TAG, "centreAlert:%d dist:%d", gridAlerts[i], primary_distance);

    centreAlert ++;
}

}


                    }
                } // for every square

// if( runCounter % 10 == 0 ) {
average_cm = distance_total/16;

if ( abs(average_cm - total_average_cm) > 50 ){
    total_average_cm = average_cm; // update total if average_cm changed
}



 ESP_LOGD(TAG, "average_cm final %d", total_average_cm);
// }
                // Calculate average and update alerts
                int average_distance = primary_distance_total/8;
                alert_level_t alert_level_average = get_alert_level(average_distance, total_average_cm);

//ESP_LOGI(TAG, "average tot:%d avg prim:%d avg all:%d", primary_distance_total, average_distance, total_average_cm);


if (centreAlert >= centreAlert_WarnLevel){
update_beeper_alerts(alert_level_t::ALERT_IMMEDIATE, 1);
ESP_LOGI(TAG, "centreAlert:%d ALERT_IMMEDIATE", centreAlert);
//centreAlert = 0;
} 
/*
else if (centreAlert > 3){
update_beeper_alerts(alert_level_t::ALERT_CLOSE, 1);
ESP_LOGI(TAG, "centreAlert:%d ALERT_CLOSE", centreAlert);

} 
*/
else {
update_beeper_alerts(alert_level_average, 1);
ESP_LOGI(TAG, "centreAlert:%d alert_level_average %d", centreAlert, alert_level_average);
}
centreAlert = 0;



      if (topAlert){
gpio_set_level(RED_PIN, 0); 
gpio_set_level(BLUE_PIN, 1);  
gpio_set_level(GREEN_PIN, 0); 
        }
       if (bottomAlert){
gpio_set_level(RED_PIN, 1); 
gpio_set_level(BLUE_PIN, 0);  
gpio_set_level(GREEN_PIN, 0); 
        }
      if (!bottomAlert && !topAlert){
gpio_set_level(RED_PIN, 0); 
gpio_set_level(BLUE_PIN, 0);  
gpio_set_level(GREEN_PIN, 0); 
      }

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
                                printf("%2d", 2);
                                break;
                            case alert_level_t::ALERT_MEDIUMFAR:
                                printf("%2d", 3);
                                break;
                            case alert_level_t::ALERT_MEDIUM:
                                printf("%2d", 4);
                                break;
                            case alert_level_t::ALERT_CLOSE:
                                printf("%2d", 5);
                                break;
                            case alert_level_t::ALERT_IMMEDIATE:
                                printf("%2d", 6);
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
            } else { // not ready
           not_ready_count++;
            ESP_LOGW(TAG, "Sensor not ready or comms error. Count: %d/%d", not_ready_count, NOT_READY_THRESHOLD);
            

            if (not_ready_count >= NOT_READY_THRESHOLD) {
                ESP_LOGE(TAG, "Not ready threshold reached. Attempting recovery...");
                not_ready_count = 0;
                 if (vl53l5cx_hasMutex){
                xSemaphoreGive(g_i2c_bus_mutex);
                ESP_LOGD("MUTEX1", "vl53l5cx g_i2c_bus Mutex given -> recovery.");
                 }
                vl53l5cx_hasMutex = false;
                // Call the new, comprehensive recovery function
                if (vl53l5cx_recover(Dev)) {
                    // Recovery was successful, reset the counter and continue.
                    
                    vTaskDelay(pdMS_TO_TICKS(50));
                } else {
                    // Catastrophic failure. The sensor couldn't be recovered.
                    // We can either try again after a long delay or stop the task.
                    ESP_LOGE(TAG, "Catastrophic recovery failure. Halting task.");
                    
                    vTaskDelay(pdMS_TO_TICKS(500));
                    //vTaskDelete(NULL); // Or vTaskDelay for a long time...
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(200)); // break, to regain ready state if not > NOT_READY_THRESHOLD
            }

            }
//VL53L5CX_WaitMs(&(Dev->platform), 20);
            // printf("-------------------\n");
        } // SENSOR_READ_INTERVAL passed
           // Periodic Stack Check (e.g., every 10 iterations)
       
        runCounter++;

 
        if( runCounter % 10 == 0 ) {
            minStackBytesRemaining = uxTaskGetStackHighWaterMark(NULL);
      //      ESP_LOGI(TAG, "Sensor Task Stack : %d bytes left", minStackBytesRemaining);
            if( minStackBytesRemaining < 100 ){
                ESP_LOGW(TAG, "Sensor Task Stack Warning: %d bytes left", minStackBytesRemaining);
            }
        }
        
     if (vl53l5cx_hasMutex){
xSemaphoreGive(g_i2c_bus_mutex);
ESP_LOGD("MUTEX1", "vl53l5cx Giving g_i2c_bus mutex...");
     }
vl53l5cx_hasMutex = false;

        // Small delay to prevent watchdog issues and allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(50));

//esp_task_wdt_feed();

               // --- CRITICAL PART: Check for a STOP signal ---
                uint32_t stop_signal_value = 0;
                // Use a zero timeout to check instantly without blocking
                xTaskNotifyWait(0, ULONG_MAX, &stop_signal_value, 0);

                // 3. If a STOP signal was sent, break the inner loop
                if (stop_signal_value & STOP_BIT) {
                    ESP_LOGI(TAG, "STOP signal received! vl53l5cx Exiting loop and going back to sleep.");
                    break; // Exit the inner "running" loop
                }

                
     } // while 1 loop end

if (vl53l5cx_hasMutex){
   xSemaphoreGive(g_i2c_bus_mutex);
      ESP_LOGD("MUTEX1", "g_i2c_bus Mutex given.");
}
 vl53l5cx_hasMutex = false;
    } //  got mutex
        else {
            vl53l5cx_hasMutex = false;
        ESP_LOGW(TAG, "No mutex available");
    //    xSemaphoreGive(g_i2c_bus_mutex);
        }


 


    }  // got semaphore  signal  



} // waiting for g_binary_semaphore

    // Cleanup (though this won't be reached in normal operation)
    vl53l5cx_stop_ranging(Dev);
  
    set_beeper_tone(0, false);
    vTaskDelete(NULL);
}
#endif

 #ifdef useUltrasound
 void ultrasonic_ranger(void *pvParameters) {

    // ultrasound_mutex

    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO,
    };

uint32_t last_update_time = xTaskGetTickCount();
static int ultrasonic_update_period = 5;
/*
  ultrasonic_sensor_t sensor = {
    .trigger_pin, //!< GPIO output pin for trigger
    .echo_pin,    //!< GPIO input pin for echo
    .minDistance = 5,
    .maxDistance = 300,
    .maxRetry = 3,
    .pulseTuning = 200,
    .debug = false,
} 
*/

  ultrasonic_init(&sensor);

 //void parameters(int index,int trigPin, int echoPin, int minDistance = 5, int maxDistance = 300, int maxRetry = 3, int pulseTuning = 200, bool debug = false);

//myObjSonar.parameters(2, trigPin3, echoPin3, 5, 300, 2, 130, true); 
/*
   gpio_num_t trigger_pin; //!< GPIO output pin for trigger
    gpio_num_t echo_pin;    //!< GPIO input pin for echo
    int minDistance = 5;
    int maxDistance = 300;
    int maxRetry = 3;
    int pulseTuning = 200;
    bool debug = false;
*/


 //   ultrasonic_init(&sensor2);



for (;;) {

            uint32_t notification_value = 0;

        // 1. Wait indefinitely for any notification (e.g., the START signal)
        // This call will block until xTaskNotify is called on this task.
        xTaskNotifyWait(0,           /* Don't clear any bits on entry */
                        ULONG_MAX,   /* Clear all bits on exit */
                        &notification_value, /* Stores the notification value */
                        portMAX_DELAY);      /* Block forever */

        // 2. Check if the START signal was received
        if (notification_value & START_BIT) {
            ESP_LOGI(TAG, "START signal for ultrasonic reading loop 1.");

//static int sensornumber = 0;
//ultrasonic_sensor_t active_sensor = sensor;
     uint32_t distance = 0;
     uint32_t distance2 = 0;

float temp = 25.0;
// 

    while (true)
    {
   

//uint32_t active_distance = distance;

//if (sensornumber == 0){
//ultrasonic_sensor_t active_sensor = sensor;
//active_distance = distance;
//sensornumber = 1;
//} else {
//ultrasonic_sensor_t active_sensor = sensor2;
//active_distance = distance2;
//ensornumber = 0;
//}  uint32_t current_time = xTaskGetTickCount();


uint32_t current_time = xTaskGetTickCount();
 last_update_time = 0;

  if (xSemaphoreTake(ultrasound_mutex, portMAX_DELAY) == pdTRUE) {
 ESP_LOGD(TAG, "ultrasound Got mutex .");
            alert_level_t alert_level = ALERT_VERYFAR;
    ///    alert_level_t alert_level_2 = ALERT_VERYFAR;



//if (current_time - last_update_time > ultrasonic_update_period ) {
// last_update_time = current_time;


uint32_t u1_read_time = xTaskGetTickCount();
        esp_err_t res = ultrasonic_measure_cm_temp_compensated(&sensor, MAX_DISTANCE_CM, &distance, temp);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    alert_level = get_alert_level( 500, 200);
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else {
       //     printf("Distance from sensor %d: %lu\n", 1, distance);
            alert_level = get_alert_level( distance, 200);
        }



//ESP_LOGI(TAG, "alert_level dist: %lu ",  distance);
update_beeper_alerts(alert_level, 1);


/*
vTaskDelay(pdMS_TO_TICKS(400)); // wait for signal from sensor1 to die out

////// sensor 2

        res = ultrasonic_measure_cm(&sensor2, MAX_DISTANCE_CM, &distance2);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    alert_level_2 = get_alert_level( 500, 200);
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else {
           // printf("Distance from sensors 1:%lu 2:%lu\n",  distance, distance2);
         //   alert_level_2 = get_alert_level( distance2, 200);
        }
*/

              
//ESP_LOGI(TAG, "alert_level dist: %lu ",  distance);
//  if (distance2 <= distance) {
  //      alert_level_2 = get_alert_level( distance2, 200);
   //     update_beeper_alerts(alert_level_2, 2);
//  } else {
        alert_level = get_alert_level( distance, 200);
        update_beeper_alerts(alert_level, 1);
//  }


vTaskDelay(pdMS_TO_TICKS(200)); // wait for signal from sensor1 to die out
////// sens 2
     uint32_t distance2 = 0;
   alert_level_t alert_level_2 = ALERT_VERYFAR;


       ultrasonic_sensor_t sensor2 = {
        .trigger_pin = TRIGGER_GPIO_2,
        .echo_pin = ECHO_GPIO_2,
    };
    ultrasonic_init(&sensor2);

uint32_t  min_time_between_ultrasound = 50;
uint32_t u2_read_time = xTaskGetTickCount();

while ( (u2_read_time - u1_read_time) < min_time_between_ultrasound){
vTaskDelay(pdMS_TO_TICKS(20));
u2_read_time = xTaskGetTickCount();
 ESP_LOGI(TAG, "waiting for %d - %d > %d", u2_read_time, u1_read_time, min_time_between_ultrasound );
}
       res = ultrasonic_measure_cm_temp_compensated(&sensor2, MAX_DISTANCE_CM, &distance2, temp);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    alert_level_2 = get_alert_level( 500, 200);
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else {
           // printf("Distance from sensors 1:%lu 2:%lu\n",  distance, distance2);
         //   alert_level_2 = get_alert_level( distance2, 200);
         
        }

    printf("Distance from sensors 1:%lu 2:%lu\n",  distance, distance2);
                //  printf("Distance from sensors 2:%lu\n",  distance2);
//ESP_LOGI(TAG, "alert_level dist: %lu ",  distance);
//  if (distance2 <= distance) {
        alert_level_2 = get_alert_level( distance2, 200);
        update_beeper_alerts(alert_level_2, 2);
//  } else {
     //   alert_level = get_alert_level( distance, 200);
      //  update_beeper_alerts(alert_level, 1);
//  }


xSemaphoreGive(ultrasound_mutex);
 ESP_LOGD(TAG, "ultrasound gave mutex .");
//vTaskDelay(pdMS_TO_TICKS(100));
       // vTaskDelay(pdMS_TO_TICKS(500));

        
//            } //update time


} // // got ultrasound_mutex 
else {
      ESP_LOGI(TAG, "No mutex for ultra 1."); 
}

               // --- CRITICAL PART: Check for a STOP signal ---
                uint32_t stop_signal_value = 0;
                // Use a zero timeout to check instantly without blocking
                xTaskNotifyWait(0, ULONG_MAX, &stop_signal_value, 0);

                // 3. If a STOP signal was sent, break the inner loop
                if (stop_signal_value & STOP_BIT) {
                    ESP_LOGI(TAG, "STOP signal for ultrasound loop.");
                    break; // Exit the inner "running" loop
                }
vTaskDelay(pdMS_TO_TICKS(400));
    }/// loop while true
    


} // start semaphore

} // outer loop waiting for semaphore

}
#endif


 #ifdef useUltrasound_2
 void ultrasonic_ranger_2(void *pvParameters) {

// ultrasound_mutex

uint32_t last_update_time = xTaskGetTickCount();
static int ultrasonic_update_period = 5;

 //void parameters(int index,int trigPin, int echoPin, int minDistance = 5, int maxDistance = 300, int maxRetry = 3, int pulseTuning = 200, bool debug = false);

//myObjSonar.parameters(2, trigPin3, echoPin3, 5, 300, 2, 130, true); 
       ultrasonic_sensor_t sensor2 = {
        .trigger_pin = TRIGGER_GPIO_2,
        .echo_pin = ECHO_GPIO_2,
    };
    ultrasonic_init(&sensor2);



for (;;) {

            uint32_t notification_value = 0;

        // 1. Wait indefinitely for any notification (e.g., the START signal)
        // This call will block until xTaskNotify is called on this task.
        xTaskNotifyWait(0,           /* Don't clear any bits on entry */
                        ULONG_MAX,   /* Clear all bits on exit */
                        &notification_value, /* Stores the notification value */
                        portMAX_DELAY);      /* Block forever */

        // 2. Check if the START signal was received
        if (notification_value & START_BIT) {
            ESP_LOGI(TAG, "START signal for ultrasonic reading loop 2.");


     uint32_t distance2 = 0;

// 
   alert_level_t alert_level_2 = ALERT_VERYFAR;
   
    while (true)
    {
   
    
     

//uint32_t active_distance = distance;

//if (sensornumber == 0){
//ultrasonic_sensor_t active_sensor = sensor;
//active_distance = distance;
//sensornumber = 1;
//} else {
//ultrasonic_sensor_t active_sensor = sensor2;
//active_distance = distance2;
//ensornumber = 0;
//}  uint32_t current_time = xTaskGetTickCount();
uint32_t current_time = xTaskGetTickCount();
last_update_time = 0;


 if (xSemaphoreTake(ultrasound_mutex, portMAX_DELAY) == pdTRUE) {
 ESP_LOGI(TAG, "ultrasound2 Got mutex .");
////// sensor 2


if (current_time - last_update_time > ultrasonic_update_period ) {
// last_update_time = current_time;

float temp = 25.0;
 

       esp_err_t res = ultrasonic_measure_cm_temp_compensated(&sensor2, MAX_DISTANCE_CM, &distance2, temp);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    alert_level_2 = get_alert_level( 500, 200);
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else {
           // printf("Distance from sensors 1:%lu 2:%lu\n",  distance, distance2);
         //   alert_level_2 = get_alert_level( distance2, 200);
         
        }


                  printf("Distance from sensors 2:%lu\n",  distance2);
//ESP_LOGI(TAG, "alert_level dist: %lu ",  distance);
//  if (distance2 <= distance) {
        alert_level_2 = get_alert_level( distance2, 200);
        update_beeper_alerts(alert_level_2, 2);
//  } else {
     //   alert_level = get_alert_level( distance, 200);
      //  update_beeper_alerts(alert_level, 1);
//  }

xSemaphoreGive(ultrasound_mutex);
 ESP_LOGI(TAG, "ultrasound2 Gave mutex.");
//vTaskDelay(pdMS_TO_TICKS(100));
       // vTaskDelay(pdMS_TO_TICKS(500));

            } //  if (xSemaphoreTake(ultrasound_mutex, portMAX_DELAY) == pdTRUE) {


} // 

else {

    ESP_LOGI(TAG, "No mutex for ultra 2."); 
}


               // --- CRITICAL PART: Check for a STOP signal ---
                uint32_t stop_signal_value = 0;
                // Use a zero timeout to check instantly without blocking
                xTaskNotifyWait(0, ULONG_MAX, &stop_signal_value, 0);

                // 3. If a STOP signal was sent, break the inner loop
                if (stop_signal_value & STOP_BIT) {
                    ESP_LOGI(TAG, "STOP signal for ultrasound loop.");
                    break; // Exit the inner "running" loop
                }
vTaskDelay(pdMS_TO_TICKS(200));
    }/// loop while true
    


} // start semaphore

} // outside forever loop

}
#endif

/*
void hcsr04_task(void *pvParameters)
{
    esp_err_t return_value = ESP_OK;
    (void) UltrasonicInit();
// GPIO pins to HC SR04 module
//#define ESP_HCSR04_TRIGGER_PIN    CONFIG_TRIGGER_PIN   // define trigger IO pin 
//#define ESP_HCSR04_ECHO_PIN       CONFIG_ECHO_PIN      // define echo IO pin 

    
    // create variable which stores the measured distance
    static uint32_t afstand = 0;

    while (1) {
        return_value = UltrasonicMeasure(100, &afstand);
        UltrasonicAssert(return_value);
        if (return_value == ESP_OK) {
            printf ("Afstand: %ld\n", afstand);
             ESP_LOGI("ULTRA", "Afstand %ld", afstand);
        }   else {

    //        gpio_set_direction(ESP_HCSR04_TRIGGER_PIN, GPIO_MODE_OUTPUT);
    //gpio_set_direction( ESP_HCSR04_ECHO_PIN, GPIO_MODE_INPUT);
             ESP_LOGI("ULTRA", "hcsr04_task problem %x\n", return_value); //, ESP_HCSR04_TRIGGER_PIN, ESP_HCSR04_ECHO_PIN);
           
     //   ESP_LOGI(log_tag, "Measurement error: %x\n", return_value);
        }  
    
        // 0,5 second delay before starting new measurement
        vTaskDelay(500 / portTICK_PERIOD_MS);
    } 
}
*/

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
 #ifdef useVibrator
     esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_MSEC * 1000,
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,    // Bitmask of all cores
        .trigger_panic = false,
    };
#endif

 #ifdef useGPS_2  
    nmea_example_init_interface();
   read_and_parse_nmea();
#endif

gpio_set_direction(RED_PIN, GPIO_MODE_OUTPUT);
gpio_set_direction(BLUE_PIN, GPIO_MODE_OUTPUT);
gpio_set_direction(GREEN_PIN, GPIO_MODE_OUTPUT);
//gpio_set_level(RED_PIN, 0); 
//gpio_set_level(BLUE_PIN, 0);  
//gpio_set_level(GREEN_PIN, 0); 


gpio_set_direction(RED_PIN_1, GPIO_MODE_OUTPUT);
gpio_set_direction(BLUE_PIN_1, GPIO_MODE_OUTPUT);
gpio_set_direction(GREEN_PIN_1, GPIO_MODE_OUTPUT);
//gpio_set_level(RED_PIN_1, 0); 
//gpio_set_level(BLUE_PIN_1, 0);  
//gpio_set_level(GREEN_PIN_1, 0); 

//gpio_set_direction(VIBRATORPIN, GPIO_MODE_OUTPUT); 
//gpio_set_level(VIBRATORPIN, 0); 



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

 #ifdef useGPS    
            uart_mutex = xSemaphoreCreateMutex();
    if (uart_mutex == NULL) {
        ESP_LOGE("MAIN", "Fatal: Failed to create UART  mutex!");
        return;
    }
#endif

 #ifdef useUltrasound    
            ultrasound_mutex = xSemaphoreCreateMutex();
    if (ultrasound_mutex == NULL) {
        ESP_LOGE("MAIN", "Fatal: Failed to create Ultrasound mutex!");
        return;
    }
#endif

// 1. Declare a global handle for the semaphore
//SemaphoreHandle_t g_binary_semaphore;


#ifdef useAccelerometer
        mpu_data_queue = xQueueCreate(5, sizeof(mpu_data_t));
#endif

led_state_queue = xQueueCreate(1, sizeof(led_state_t)); // Correct queue
ESP_LOGI("MAIN", "Mutex and Queues created.");

#ifdef useRGBLed
configure_led();
//rgb_led_init();
vTaskDelay(pdMS_TO_TICKS(100));

#endif

#ifdef useGPS
GPS_init();

xTaskCreatePinnedToCore(
                    gps_task,   /* Function to implement the task */
                    "gps_task", /* Name of the task */
                    1024 * 3,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    4,          /* Priority of the task */
                    &xHandle_GPS,       /* Task handle. */
                    0);  /* Core where the task should run */
 

 //   xTaskCreate(&gps_task, "gps_task", 1024 * 3, NULL, 5, &xHandle_GPS);
#endif


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

#ifdef useVl53l5cx
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

    // Prepare task parameters
    static sensor_task_params_t task_params = {
        .dev = &Dev,
        .gridLayout = gridLayout,
        .gridAlerts = gridAlerts
    };
#endif

 // --- 5. Create and Launch All Tasks ---
    ESP_LOGI("MAIN", "Creating tasks...");

#ifdef useRGBLed
    ESP_LOGI("MAIN", "Creating task for RGB LED");
    // Create the LED task. It's now safe to start.
    xTaskCreate(led_control_task, "LED Control", 4096, NULL, 4, NULL);
#endif


#ifdef useUltrasound
 // xTaskCreate(hcsr04_task, "HSRC04 task", 2048, NULL, 4, NULL);
 xTaskCreatePinnedToCore(
                    ultrasonic_ranger,   /* Function to implement the task */
                    "ultrasonic_ranger", /* Name of the task */
                    8192,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    5,          /* Priority of the task */
                    &xHandle_ultrasonic,       /* Task handle. */
                    1);  /* Core where the task should run */
#endif
//ultrasonic_ranger_2
#ifdef useUltrasound_2
xTaskCreatePinnedToCore(
                    ultrasonic_ranger_2,   /* Function to implement the task */
                    "ultrasonic_ranger_2", /* Name of the task */
                    8192,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    5,          /* Priority of the task */
                    &xHandle_ultrasonic_2,       /* Task handle. */
                    0);  /* Core where the task should run */
  //    xTaskCreate(ultrasonic_ranger, "ultrasonic_ranger", 8192 , NULL, 5, &xHandle_ultrasonic);
#endif

#ifdef useUartA02YYUW
    init_uart();
    xTaskCreate(uart_task, "UART Task", 4096, NULL, 5, NULL);
#endif

    // Create sensor and data processing tasks
#ifdef useAccelerometer
ESP_LOGI("MAIN", "Creating task for Accelerometer");

    xTaskCreate(mpu9250_reader_task, "MPU Reader", 8192, mpu9250_dev_handle, 4, &xHandle_mpu);


    xTaskCreate(data_processor_task, "Data Processor", 8192, NULL, 3, &xHandle_mpu_processor);
    #endif

// Static storage for task parameters
//static blink_task_params_t task_params;
#ifdef useVl53l5cx
    xTaskCreate(vl53l5cx_reader_task, "VL53L5CX Reader", 8192, &task_params, 5, &xHandle_vl53l5cx);
#endif

//vl53l5cx_running = false;
///// vibration

//for (int i = 0; i < SAMPLE_CNT; ++i)
 #ifdef useVibrator
 example_ledc_init();
uint32_t duty = 8000;
int count = 0;

       while (count < 10)
    {
    
   // LEDC_DUTY = 2000;

    //#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
 vTaskDelay(1000 / portTICK_PERIOD_MS);
 duty += 100;
        count++;
    }
 #endif

 /*
ESP_LOGI(VLTAG, "Adding vl53l1x sensor to I2C bus...");
//i2c_master_bus_add_device(bus_handle, &vl53_dev_cfg, &Dev.platform.handle);

  if (vl53l1xInit(&dev, &bus_handle))
    {
        ESP_LOGI(VLTAG,"Lidar Sensor VL53L1X [OK]");
    }
    else
    {
        ESP_LOGI(VLTAG,"Lidar Sensor VL53L1X [FAIL]");
        return;
    }

    VL53L1_StopMeasurement(&dev);
    VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_MEDIUM);
    VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);

     while(1){

        VL53L1_StartMeasurement(&dev);

        while (dataReady == 0)
        {
            status = VL53L1_GetMeasurementDataReady(&dev, &dataReady);
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
        range = rangingData.RangeMilliMeter;

ESP_LOGI(VLTAG, "Measuremt range: %d", range);

        VL53L1_StopMeasurement(&dev);    

        VL53L1_StartMeasurement(&dev);

        ESP_LOGI(TAG,"Distance %d mm",range);

        vTaskDelay(pdMS_TO_TICKS(5000));

    }

    */
/*

    pwm_config_t config = {
        .period = 1000000 / PWM_VIBRATION_MOTOR_FREQ,  // Period in microseconds
        .duty = PWM_VIBRATION_MOTOR_DUTY_MIN,          // Initial duty cycle
        .delay = 0,                                  // Delay in microseconds (not used here)
        .mode = PWM_MODE革命_SIM,                    //Cumulative mode for standard PWM
        .s_q = 0,                                    // No queue (direct allocation)
    };

    // **Set GPIO as PWM output**
    gpio_pad_select_gpio(PWM_VIBRATION_MOTOR_GPIO);
    gpio_set_direction(PWM_VIBRATION_MOTOR_GPIO, GPIO_MODE_OUTPUT);
    
    // **Initialize and start PWM**
    esp_err_t err = pwm_init(&config, PWM_VIBRATION_MOTOR_CHAN, PWM_VIBRATION_MOTOR_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PWM Init failed: 0x%x", err);
        return;
    }
    pwm_start(PWM_VIBRATION_MOTOR_CHAN);

    // **Example: Set duty cycle to 50% (adjust as needed)**
    pwm_set_duty(PWM_VIBRATION_MOTOR_CHAN, 127); // 50% duty cycle
    pwm_set_frequency(PWM_VIBRATION_MOTOR_CHAN, PWM_VIBRATION_MOTOR_FREQ);
int count = 0;
       while (count < 5) {
        // **SIMPLE DEMO: Toggle between 25%, 50%, and 75% duty cycles every 500ms**
        static uint8_t duty[] = {64, 127, 191}; // 25%, 50%, 75%
        static uint8_t index = 0;
        pwm_set_duty(PWM_VIBRATION_MOTOR_CHAN, duty[index]);
        index = (index + 1) % 3; // Cycle through duty array
        vTaskDelay(pdMS_TO_TICKS(500));
        count++;
    }

*/

// blink_task_init(5, 500); // Blink 5 times with 500 ms interval

}
