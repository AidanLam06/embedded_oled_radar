#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/queue.h"

#define SERVO_CHANNEL LEDC_CHANNEL_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_OUT (2)
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_DUTY_RES LEDC_TIMER_14_BIT
#define LEDC_FREQUENCY (50)

#define PING_TIMEOUT 6000
#define TRIG_HIGH_DELAY 10
#define TRIG_LOW_DELAY 4
#define TRIG_PIN 5
#define ECHO_PIN 4

#define true 1
#define false 0

static const char *tag = "MAIN";

const int step = 13;
const int min_duty = 409; // floor(0.001/0.02 * 2^14)
const int max_duty = 1966; // floor(0.002/0.02 * 2^14)
int duty = min_duty;
bool pos_direction = true;
const int iteration_time = 10;

SemaphoreHandle_t servo_step;
QueueHandle_t ultra_to_oled;
static SemaphoreHandle_t echo_semaphore;
static int64_t echo_start = 0;
static int64_t echo_end = 0;

typedef struct {
    gpio_port_t echo_pin;
    gpio_port_t trig_pin;
} ultrasonic_sensor_t;

typedef struct {
    ultrasonic_sensor_t *sensor;
    int64_t *time_us;
} sensor_ping_task_args_t;

typedef struct {
    int distance;
} scan_data_t;

/*
Some things to try/consider:
- make program activation to allow future use of a remote activation using some web page or something --> probably done by having a function that starts the program which can be called by a listening function
- Buy an IR emitter that will allow an IR receiver to receive IR signals and move the motor using a remote --> Might have to buy another ESP32 for this since they are pretty cheap to make a remote, maybe even a cheap 3D printer to make a plastic remote body
*/

static void ledc_setup() {
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = SERVO_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_OUT,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
}

static void gpio_setup() {
    gpio_config_t trig_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<TRIG_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&trig_config);

    gpio_config_t echo_config = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<ECHO_PIN),
        .pull_down_en = 0,
        .pull_down_en = 0
    };
    gpio_config(&echo_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO_PIN, echo_isr_handler, NULL);
}

void servo_loop_task(void *args) {
    int step = 15;
    int min_duty = 409; // for 14 bit resolution
    int max_duty = 1966; // for 14 bit resolution
    int duty = min_duty;
    bool pos_direction = true;
    int iteration_time = 10;

    for (;;) {
        if (pos_direction) duty += step;
        else duty -= step;

        if (duty >= max_duty) pos_direction = false;
        if (duty <= min_duty) pos_direction = true;

        ledc_set_duty(LEDC_MODE, SERVO_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, SERVO_CHANNEL);

        vTaskDelay(iteration_time / portTICK_PERIOD_MS);
    }
}    

void servo_step_task(void *args) { // will trigger a single step in the servo every time it is called. I think iteration time should be applied in the main loop.
    for (;;) {
        if (pos_direction) duty += step;
        else duty -= step;

        if (duty >= max_duty) pos_direction = false;
        if (duty <= min_duty) pos_direction = true;

        ledc_set_duty(LEDC_MODE, SERVO_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, SERVO_CHANNEL);

        xSemaphoreGive(servo_step);

        vTaskDelay(iteration_time / portTICK_PERIOD_MS);
    }
}

void ultrasonic_init(const ultrasonic_sensor_t *dev) {
    gpio_set_direction(dev->trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->echo_pin, GPIO_MODE_INPUT);
}

static void IRAM_ATTR echo_isr_handler(void *args) {
    int level = gpio_get_level(ECHO_PIN);
    int64_t time_now = esp_timer_get_time();

    if (level == 1) 
        echo_start = time_now;
    else {
        echo_end = time_now;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(echo_semaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

void sensor_ping_task(void *pvParameters) {
    // later add some additional interrupt logic to block interrupts during sensor ping

    sensor_ping_task_args_t *args = (sensor_ping_task_args_t *)pvParameters;
    ultrasonic_sensor_t *sensor = args->sensor;
    int64_t *time_us = args->time_us;

    for (;;) {
        if (xSemaphoreTake(servo_step, portMAX_DELAY)) {
            
            gpio_set_level(sensor->trig_pin, 0);
            esp_rom_delay_us(TRIG_LOW_DELAY);
            gpio_set_level(sensor->trig_pin,1);
            esp_rom_delay_us(TRIG_HIGH_DELAY);
            gpio_set_level(sensor->trig_pin, 0);

            if (xSemaphoreTake(echo_semaphore, pdMS_TO_TICKS(PING_TIMEOUT))) {
                int64_t duration = echo_end - echo_start;
                ESP_LOGI(tag, "asd");
                // add queue logic here for the ultra_to_oled queue, since this is the giver and the oled is the reciever
            }
        }   
    }
}

void write_to_oled_task(void *args) {
    printf("Finna write to the oled");
}

void app_main(void) {
    static int64_t time;
    int64_t roundtrip_time;
    int64_t distance = roundtrip_time/58.3;

    echo_semaphore = xSemaphoreCreateBinary();
    servo_step = xSemaphoreCreateBinary();

    static ultrasonic_sensor_t sensor = {
        .trig_pin = TRIG_PIN,
        .echo_pin = ECHO_PIN
    };

    static sensor_ping_task_args_t args = {
        .sensor = &sensor,
        .time_us = &time
    };
    
    ledc_setup();
    gpio_setup();

    ultrasonic_init(&sensor);

    xTaskCreate(&servo_step_task, "servo_step_task", 2048, NULL, 5, NULL);
    xTaskCreate(&sensor_ping_task, "sensor_ping_task", 2048, &args, 5, NULL);
}