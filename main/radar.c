#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/ledc.h"

#define SERVO_CHANNEL LEDC_CHANNEL_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_OUT (2)
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_DUTY_RES LEDC_TIMER_14_BIT
#define LEDC_FREQUENCY (50)

#define PING_TIMEOUT 6000
#define TRIG_HIGH_DELAY 10
#define TRIG_LOW_DELAY 4
#define TRIG_PIN 4
#define ECHO_PIN 5

#define true 1
#define false 0

static const char *tag = "MAIN";

const int step = 13;
const int min_duty = 409; // floor(0.001/0.02 * 2^14)
const int max_duty = 1966; // floor(0.002/0.02 * 2^14)
int duty = min_duty;
bool pos_direction = true;
const int iteration_time = 10;



typedef struct {
    gpio_port_t echo_pin;
    gpio_port_t trig_pin;
} ultrasonic_sensor_t;

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
    if (pos_direction) duty += step;
    else duty -= step;

    if (duty >= max_duty) pos_direction = false;
    if (duty <= min_duty) pos_direction = true;

    ledc_set_duty(LEDC_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, SERVO_CHANNEL);

    vTaskDelay(iteration_time / portTICK_PERIOD_MS);
}

void ultrasonic_init(const ultrasonic_sensor_t *dev) {

}

void sensor_ping_task(void *args) {
    // later add some additional interrupt logic to block interrupts during sensor ping
    ultrasonic_sensor_t sensor = {
        .trig_pin = TRIG_PIN,
        .echo_pin = ECHO_PIN
    };

    ultrasonic_sensor_t *sensor_ptr = &sensor;

    gpio_set_level(sensor_ptr->trig_pin, 0);
    ets_delay_us(TRIG_LOW_DELAY);
    gpio_set_level(sensor_ptr->trig_pin,1);
    ets_delay_us(TRIG_HIGH_DELAY);
    gpio_set_level(sensor_ptr->trig_pin, 0);

    if (gpio_get_level(sensor_ptr->echo_pin))
        ESP_LOGE(tag, "ECHO already HIGH");

    uint64_t start = esp_timer_get_time();
    while (!gpio_get_level(sensor_ptr->echo_pin)) { // busy cycle until echo_pin HIGH
        if (esp_timer_get_time()-start > PING_TIMEOUT)
            ESP_LOGE(tag, "Sensor _Ping_ Timeout");
    }

    uint64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(sensor_ptr->echo_pin)) {
        if (esp_timer_get_time()-start > PING_TIMEOUT)
            ESP_LOGE(tag, "Sensor _Echo_ Timeout");
    }


}

void app_main(void) {    
    ledc_setup();

    xTaskCreate(&servo_loop_task, "servo_loop_task", 2048, NULL, 5, NULL);
}