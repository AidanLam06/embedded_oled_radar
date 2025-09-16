#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "freertos/queue.h"
#include "driver/i2c.h"

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
#define MAX_SCAN_DISTANCE 200 // 200 cm, 2m
#define CENTER_X 64
#define CENTER_Y 64


#define I2C_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 800000

#define PIXEL_LIFETIME_MS 750

#define true 1
#define false 0


static const char *tag_main = "MAIN";
static const char *tag_ssd1306 = "SSD1306";

const int step = 9;
const int min_duty = 409; // floor(0.001/0.02 * 2^14)
const int max_duty = 1966; // floor(0.002/0.02 * 2^14)
int duty = min_duty;
bool pos_direction = true;
int step_count = 0;
const int max_steps = (max_duty-min_duty)/step;
const int iteration_time = 8;

const uint8_t i2c_address = 0x3C; //address on the oled pcb is 0x78 which translates to 0x3C if you shift all the bits right by one bit

SemaphoreHandle_t servo_to_ultra;
QueueHandle_t ultra_to_oled;
SemaphoreHandle_t oled_to_servo;

// echo signal stuff
static SemaphoreHandle_t echo_semaphore;
static int64_t echo_start = 0;
static int64_t echo_end = 0;

int64_t distance_history[max_steps];
uint8_t frame_buffer[1024];
uint32_t pixel_expiration[max_steps];

typedef struct {
    gpio_port_t echo_pin;
    gpio_port_t trig_pin;
} ultrasonic_sensor_t;

typedef struct {
    ultrasonic_sensor_t *sensor;
    int64_t *time_us;
} sensor_ping_task_args_t;

typedef struct {
    int x;
    int y;
} pair_t;

// precomputed lookup table for (x2,y2) pairs
pair_t coord_lookup_table[] = {
    {2,64}, {2,63}, {2,62}, {2,61}, {2,60}, {2,58}, {2,57}, {2,56}, {3,55}, {3,54}, {3,53}, 
    {3,52}, {3,51}, {4,50}, {4,48}, {4,47}, {5,46}, {5,45}, {5,44}, {6,43}, {6,42}, {6,41}, 
    {7,40}, {7,39}, {8,38}, {8,37}, {9,36}, {9,35}, {10,34}, {10,33}, {11,32}, {12,31}, 
    {12,30}, {13,29}, {13,28}, {14,27}, {15,26}, {15,25}, {16,25}, {17,24}, {18,23}, {18,22}, 
    {19,21}, {20,20}, {21,20}, {22,19}, {22,18}, {23,17}, {24,17}, {25,16}, {26,15}, {27,14}, 
    {28,14}, {29,13}, {29,12}, {30,12}, {31,11}, {32,11}, {33,10}, {34,10}, {35,9}, {36,9}, 
    {37,8}, {38,8}, {39,7}, {40,7}, {41,6}, {42,6}, {44,5}, {45,5}, {46,5}, {47,4}, {48,4}, 
    {49,4}, {50,4}, {51,3}, {52,3}, {53,3}, {54,3}, {56,3}, {57,2}, {58,2}, {59,2}, {60,2}, 
    {61,2}, {62,2}, {63,2}, {65,2}, {66,2}, {67,2}, {68,2}, {69,2}, {70,2}, {71,2}, {72,3}, 
    {73,3}, {75,3}, {76,3}, {77,3}, {78,4}, {79,4}, {80,4}, {81,4}, {82,5}, {83,5}, {84,5}, 
    {85,6}, {86,6}, {88,7}, {89,7}, {90,8}, {91,8}, {92,8}, {93,9}, {94,10}, {95,10}, {96,11}, 
    {97,11}, {98,12}, {98,12}, {99,13}, {100,14}, {101,14}, {102,15}, {103,16}, {104,17}, 
    {105,17}, {106,18}, {106,19}, {107,20}, {108,20}, {109,21}, {110,22}, {110,23}, {111,24}, 
    {112,24}, {112,25}, {113,26}, {114,27}, {115,28}, {115,29}, {116,30}, {116,31}, {117,32}, 
    {118,33}, {118,34}, {119,35}, {119,36}, {120,37}, {120,38}, {121,39}, {121,40}, {122,41}, 
    {122,42}, {122,43}, {123,44}, {123,45}, {123,46}, {124,47}, {124,48}, {124,49}, {125,51}, 
    {125,52}, {125,53}, {125,54}, {125,55}, {125,56}, {126,57}, {126,58}, {126,59}, {126,61}, 
    {126,62}, {126,63}, {126,64}
};

// ====================================== ISR =======================================

// for the echo
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

// ==================================================================================

// ====================================== INIT ======================================
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
        .pull_up_en = 0
    };
    gpio_config(&echo_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO_PIN, echo_isr_handler, NULL);
}

static void i2c_master_init(int16_t sda, int16_t scl, int16_t reset) {
    i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = sda,
		.scl_io_num = scl,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ
	};
	i2c_param_config(I2C_NUM, &i2c_config);
	i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

static void i2c_init(i2c_port_t i2c_num, uint8_t oled_address) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (oled_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Control byte: following are commands

    i2c_master_write_byte(cmd, 0xAE, true); // Display OFF
    i2c_master_write_byte(cmd, 0x20, true); // Set Memory Addressing Mode
    i2c_master_write_byte(cmd, 0x02, true); // Page addressing mode
    i2c_master_write_byte(cmd, 0xB0, true); // Set Page Start Address (Page 0)
    i2c_master_write_byte(cmd, 0xC8, true); // COM Output Scan Direction remapped
    i2c_master_write_byte(cmd, 0x00, true); // Low column address
    i2c_master_write_byte(cmd, 0x10, true); // High column address
    i2c_master_write_byte(cmd, 0x40, true); // Set Display Start Line
    i2c_master_write_byte(cmd, 0x81, true); // Set Contrast
    i2c_master_write_byte(cmd, 0x7F, true); //   (mid level)
    i2c_master_write_byte(cmd, 0xA1, true); // Segment re-map
    i2c_master_write_byte(cmd, 0xA6, true); // Normal display (not inverted)
    i2c_master_write_byte(cmd, 0xA8, true); // Set multiplex ratio
    i2c_master_write_byte(cmd, 0x3F, true); //   (1/64 duty for 128x64)
    i2c_master_write_byte(cmd, 0xA4, true); // Resume display from RAM
    i2c_master_write_byte(cmd, 0xD3, true); // Set display offset
    i2c_master_write_byte(cmd, 0x00, true); //   no offset
    i2c_master_write_byte(cmd, 0xD5, true); // Set display clock divide
    i2c_master_write_byte(cmd, 0x80, true); //   suggested ratio
    i2c_master_write_byte(cmd, 0xD9, true); // Set pre-charge
    i2c_master_write_byte(cmd, 0xF1, true);
    i2c_master_write_byte(cmd, 0xDA, true); // Set COM pins config
    i2c_master_write_byte(cmd, 0x12, true);
    i2c_master_write_byte(cmd, 0xDB, true); // Set VCOMH deselect
    i2c_master_write_byte(cmd, 0x40, true);
    i2c_master_write_byte(cmd, 0x8D, true); // Charge pump
    i2c_master_write_byte(cmd, 0x14, true); //   enable
    i2c_master_write_byte(cmd, 0xAF, true); // Display ON

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void ultrasonic_init(const ultrasonic_sensor_t *dev) {
    gpio_set_direction(dev->trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->echo_pin, GPIO_MODE_INPUT);
}

// ==================================================================================

// ===================================== TASKS ======================================
void servo_loop_task(void *args) {
    // While this task is not useful for the final product of this program, it works, so keep it until debugging is complete
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
        if (xSemaphoreTake(oled_to_servo, portMAX_DELAY)) {
            if (pos_direction) duty += step;
            else duty -= step;

            if (pos_direction) step_count++;
            else step_count--;

            if (duty >= max_duty) pos_direction = false;
            if (duty <= min_duty) pos_direction = true;

            ledc_set_duty(LEDC_MODE, SERVO_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, SERVO_CHANNEL);  

            xSemaphoreGive(servo_to_ultra);

            vTaskDelay(iteration_time / portTICK_PERIOD_MS);
        }
    }
}

void sensor_ping_task(void *pvParameters) {
    sensor_ping_task_args_t *args = (sensor_ping_task_args_t *)pvParameters;
    ultrasonic_sensor_t *sensor = args->sensor;
    int64_t *time_us = args->time_us;

    for (;;) {
        if (xSemaphoreTake(servo_to_ultra, portMAX_DELAY)) {
            gpio_set_level(sensor->trig_pin, 0);
            esp_rom_delay_us(TRIG_LOW_DELAY);
            gpio_set_level(sensor->trig_pin,1);
            esp_rom_delay_us(TRIG_HIGH_DELAY);
            gpio_set_level(sensor->trig_pin, 0);

            if (xSemaphoreTake(echo_semaphore, pdMS_TO_TICKS(PING_TIMEOUT))) {
                int64_t duration = echo_end - echo_start;
                int distance_cm = (int)(duration/58); 
                xQueueSend(ultra_to_oled, &distance_cm, portMAX_DELAY);
            }
        }   
    }
}

void write_to_oled_task(void *args) {
    uint8_t *buffer = (uint8_t *)args;
    int distance;

    TickType_t prev_refresh = xTaskGetTickCount();
    const TickType_t refresh = pdMS_TO_TICKS(33);

    for (;;) {
        int x2 = coord_lookup_table[step_count].x;
        int y2 = coord_lookup_table[step_count].y;

        draw_bresenham_semicircle(buffer); // its either small RAM overhead or even smaller CPU overhead so I chose this rahter than just making one semicircle and copying it constantly
        draw_bresenham_line(buffer, CENTER_X, CENTER_Y, x2, y2);

        if (xQueueReceive(ultra_to_oled, &distance, portMAX_DELAY)) {
            if (abs(distance - distance_history[step_count]) > 4) {
                distance_history[step_count] = distance;
                pixel_expiration[step_count] = xTaskGetTickCount() + pdMS_TO_TICKS(750);
            }

        }

        uint32_t now = xTaskGetTickCount();
        for (int step = 0; step < max_steps; step++) {
            if (pixel_expiration[step] > now) {
                draw_pixel(buffer, distance, x2, y2);
            }
        }

        for (int page = 0; page < 8; page++) {
            ssd1306_write_page(I2C_NUM_0, i2c_address, page, 0, &buffer[page*128], 128);
        }
    vTaskDelayUntil(&prev_refresh, refresh);
    }
}

// ===================================================================================

// =================================== FUNCTIONS =====================================

void draw_bresenham_line(uint8_t *buffer, int x1, int y1, int *x2_ptr, int *y2_ptr) {
    int y2 = *y2_ptr;
    int x2 = *x2_ptr;
    int m_new = 2*(y2-y1);
    int slope_error_new = m_new - (x2-x1);
    int page, bit, index;

    for (int x = x1, y = y1; x <= x2; x++) {
        slope_error_new += m_new;
        page = y/8;
        bit = y%8;
        index = 128*page + x;
        buffer[index] |= (1 << bit);

        if (slope_error_new >= 0) {
            y++;
            slope_error_new -= 2*(x2-x1);
        }
    }
}

void draw_bresenham_semicircle(uint8_t *buffer) {
    // I only need (-y,-x), (-x,-y), (x,-y), (y, -x) since I 1) only want half the circle 2) the origin (0,0) is at the top left of the screen and the max is at the bottom right so any shape meant to be plotted on a typical graph would be inverted

    int r = 62;
    int x = 0, y = r;
    int d = 3 - 2*r;
    
    while (x <= y) {
        pair_t pairs[4] = {
            {CENTER_X-y,CENTER_Y-x},
            {CENTER_X-x,CENTER_Y-y},
            {CENTER_X+x,CENTER_Y-y},
            {CENTER_X+y,CENTER_Y-x}
        };

        for (int i = 0; i<4; i++) {
            int px = pairs[i].x;
            int py = pairs[i].y;

            int page = py/8;
            int bit = py%8;
            int index = page*128 + px;
            buffer[index] |= (1<<bit);
        }

        if (d < 0) {
            d += 4*x + 6;
        } else {
            d += 4*(x-y) + 10;
            y--;
        }
        x++;
    }
}

void draw_pixel(uint8_t *buffer, int distance, int x2, int y2) {
    // inside this function there needs to be the logic to find where the pixel should be placed and then also writing that pixel to the frame_buffer
    float pix_radius_scale = distance/MAX_SCAN_DISTANCE; // this should find the position of the distance relative to the max scanning distance. Ex: distance is 50cm; 50/200 = 0.25
    // now just find the spot that would be 25% along the line to the (x2,y2) from the centerpoint
    float x_f = CENTER_X + pix_radius_scale*(x2-CENTER_X);
    float y_f = CENTER_Y + pix_radius_scale*(y2-CENTER_Y);

    int x = (int)x_f;
    int y = (int)y_f;

    int page = y/8;
    int bit = y%8;
    int index = 128*page + x;
    buffer[index] |= (1 << bit);
}

void ssd1306_write_page(i2c_port_t i2c_num, uint8_t addr, int page, int col, uint8_t *data_buffer, int len) {
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // command stream
    i2c_master_write_byte(cmd, 0x00 | (col & 0x0F), true);
    i2c_master_write_byte(cmd, 0x10 | (col >> 4), true);
    i2c_master_write_byte(cmd, 0xB0 | page, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true); // data stream
    i2c_master_write(cmd, data_buffer, len, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// ==================================================================================

void app_main(void) {
    static int64_t time;

    echo_semaphore = xSemaphoreCreateBinary();
    oled_to_servo = xSemaphoreCreateBinary();
    ultra_to_oled = xQueueCreate(8, sizeof(int));
    servo_to_ultra = xSemaphoreCreateBinary();

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
    xTaskCreate(&write_to_oled_task, "write_to_oled_task", 2048, &frame_buffer, 5, NULL);
}