#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "QUAD_SERVO_SERVER_STA";

// WiFi configuration
#define WIFI_SSID      "ESP32_HAND_CONTROLLER"
#define WIFI_PASS      "12345678"
#define PORT           1234
#define MAX_CLIENTS    1

// PCA9685 configuration
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define PCA9685_ADDR         0x40

// PCA9685 Registers
#define PCA9685_MODE1        0x00
#define PCA9685_PRESCALE     0xFE
#define PCA9685_LED0_ON_L    0x06

// Servo parameters
#define SERVO_FREQ           50
#define MIN_PULSE_US         500
#define MAX_PULSE_US         2500

// Servo channels (6 servos total)
#define SERVO_YAW_CHANNEL    0      // MPU68 Yaw
#define SERVO_PITCH_CHANNEL  1      // MPU68 Pitch
#define SERVO_2_CHANNEL      2      // Flex sensor 2
#define SERVO_ROLL_CHANNEL   4      // MPU69 Roll
#define SERVO_PITCH69_CHANNEL 5     // MPU69 Pitch
#define SERVO_6_CHANNEL      6      // Flex sensor 1

// WiFi event group
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// Servo angle lookup tables
static const int yaw_angles[256] = {
    ['L'] = 10,    // 10° (quay trái lớn)
    ['l'] = 50,    // 50° (quay trái vừa)
    ['H'] = 90,    // 90° (home)
    ['r'] = 140,   // 140° (quay phải vừa)
    ['R'] = 170    // 170° (quay phải lớn)
};

static const int pitch_angles[256] = {
    ['A'] = 180,   // 180° (pitch -20° to 5°)
    ['B'] = 150,   // 150° (pitch 20.1° to 40°)
    ['C'] = 120,   // 120° (pitch 40.1° to 55°)
    ['D'] = 90     // 90° (pitch 55.1° to 90°)
};

static const int roll_angles[256] = {
    ['X'] = 0,     // 0° (Roll -160° to -180° và 160° to 180°)
    ['Y'] = 45,    // 45° (Roll 120° to 150°)
    ['Z'] = 90,    // 90° (Roll 61° to 119°)
    ['W'] = 135,   // 135° (Roll 21° to 60°)
    ['V'] = 180    // 180° (Roll 0° to 20°)
};

static const int pitch69_angles[256] = {
    ['P'] = 180,   // 180° (chênh > +40° so với ban đầu)
    ['Q'] = 135,   // 135° (chênh > +20° so với ban đầu)
    ['U'] = 90,    // 90° (trong khoảng ±20°)
    ['T'] = 45,    // 45° (chênh < -20° so với ban đầu)
    ['S'] = 0      // 0° (chênh < -40° so với ban đầu)
};

// Flex sensor 1 -> Servo 6
static const int flex1_angles[256] = {
    ['1'] = 180,   // Dưới 100
    ['2'] = 150,   // 100-400
    ['3'] = 120,   // 400-800
    ['4'] = 90     // Trên 800
};

// Flex sensor 2 -> Servo 2
static const int flex2_angles[256] = {
    ['5'] = 60,    // Dưới 1500
    ['6'] = 120,   // 1500-1600
    ['7'] = 180    // Trên 1600
};

// Current servo angles
static int current_angles[16] = {0};  // Tất cả servo từ 0-15

// Mutex để bảo vệ shared data
static SemaphoreHandle_t angle_mutex;

// PCA9685 Functions
esp_err_t pca9685_write_byte(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void set_pwm(uint8_t channel, uint16_t on_time, uint16_t off_time) {
    if (channel > 15) return;
    
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9685_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, (on_time & 0xFF), true);
    i2c_master_write_byte(cmd, (on_time >> 8), true);
    i2c_master_write_byte(cmd, (off_time & 0xFF), true);
    i2c_master_write_byte(cmd, (off_time >> 8), true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM on channel %d", channel);
    }
    i2c_cmd_link_delete(cmd);
}

void set_servo_pulse(uint8_t channel, uint16_t pulse_us) {
    if (channel > 15) return;
    
    float pulse_length = 1000000.0 / SERVO_FREQ;
    float pulse = pulse_us;
    pulse /= pulse_length / 4096.0;
    
    uint16_t pwm_value = (uint16_t)pulse;
    set_pwm(channel, 0, pwm_value);
}

void set_servo_angle(uint8_t channel, uint8_t angle) {
    if (channel > 15) return;
    if (angle > 180) angle = 180;
    if (angle < 0) angle = 0;
    
    uint16_t pulse_us = MIN_PULSE_US + (angle * (MAX_PULSE_US - MIN_PULSE_US) / 180);
    set_servo_pulse(channel, pulse_us);
    
    // Update current angle with mutex protection
    xSemaphoreTake(angle_mutex, portMAX_DELAY);
    current_angles[channel] = angle;
    xSemaphoreGive(angle_mutex);
    
    ESP_LOGI(TAG, "Servo %d -> %d°", channel, angle);
}

// Hàm tắt tất cả các kênh PWM
void turn_off_all_channels() {
    for (int channel = 0; channel < 16; channel++) {
        set_pwm(channel, 0, 0);  // Tắt kênh bằng cách set off_time = 0
    }
    ESP_LOGI(TAG, "All PWM channels turned off");
}

// Initialize PCA9685
esp_err_t pca9685_init() {
    ESP_LOGI(TAG, "Initializing PCA9685...");
    
    // Reset device
    pca9685_write_byte(PCA9685_MODE1, 0x00);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Enable auto-increment and restart
    pca9685_write_byte(PCA9685_MODE1, 0x20);
    
    // Set prescaler for 50Hz
    uint8_t prescale = (uint8_t)((25000000.0 / (4096.0 * SERVO_FREQ)) - 0.5);
    ESP_LOGI(TAG, "Prescale value: %d", prescale);
    pca9685_write_byte(PCA9685_PRESCALE, prescale);
    
    // Enable auto-increment and restart
    pca9685_write_byte(PCA9685_MODE1, 0xA0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "PCA9685 initialized");
    return ESP_OK;
}

// I2C initialization
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized successfully");
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi station started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, trying to reconnect...");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// WiFi initialization (STA mode)
static void wifi_init_sta(void) {
    ESP_LOGI(TAG, "Initializing WiFi in STA mode...");
    
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization complete");
    ESP_LOGI(TAG, "Connecting to AP: %s", WIFI_SSID);
}

// Process received character
static void process_char(char c, int client_sock) {
    ESP_LOGI(TAG, "Received char: '%c'", c);
    
    // MPU68 Yaw (Servo 0)
    if (c == 'L' || c == 'l' || c == 'H' || c == 'r' || c == 'R') {
        int angle = yaw_angles[(int)c];
        set_servo_angle(SERVO_YAW_CHANNEL, angle);
    }
    // MPU68 Pitch (Servo 1)
    else if (c == 'A' || c == 'B' || c == 'C' || c == 'D') {
        int angle = pitch_angles[(int)c];
        set_servo_angle(SERVO_PITCH_CHANNEL, angle);
    }
    // Flex sensor 2 -> Servo 2
    else if (c == '5' || c == '6' || c == '7') {
        int angle = flex2_angles[(int)c];
        set_servo_angle(SERVO_2_CHANNEL, angle);
    }
    // MPU69 Roll (Servo 4)
    else if (c == 'X' || c == 'Y' || c == 'Z' || c == 'W' || c == 'V') {
        int angle = roll_angles[(int)c];
        set_servo_angle(SERVO_ROLL_CHANNEL, angle);
    }
    // MPU69 Pitch (Servo 5)
    else if (c == 'P' || c == 'Q' || c == 'U' || c == 'T' || c == 'S') {
        int angle = pitch69_angles[(int)c];
        set_servo_angle(SERVO_PITCH69_CHANNEL, angle);
    }
    // Flex sensor 1 -> Servo 6
    else if (c == '1' || c == '2' || c == '3' || c == '4') {
        int angle = flex1_angles[(int)c];
        set_servo_angle(SERVO_6_CHANNEL, angle);
    }
    // Status request
    else if (c == '?') {
        xSemaphoreTake(angle_mutex, portMAX_DELAY);
        char status[128];
        snprintf(status, sizeof(status), 
                 "S0:%d,S1:%d,S2:%d,S4:%d,S5:%d,S6:%d\n", 
                 current_angles[SERVO_YAW_CHANNEL],
                 current_angles[SERVO_PITCH_CHANNEL],
                 current_angles[SERVO_2_CHANNEL],
                 current_angles[SERVO_ROLL_CHANNEL],
                 current_angles[SERVO_PITCH69_CHANNEL],
                 current_angles[SERVO_6_CHANNEL]);
        xSemaphoreGive(angle_mutex);
        send(client_sock, status, strlen(status), 0);
    }
    else {
        ESP_LOGW(TAG, "Unknown character: '%c' (0x%02x)", c, c);
    }
}

// TCP server task
static void tcp_server_task(void *pvParameters) {
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t addr_len;
    int client_sock = -1;
    char rx_char;
    int len;
    
    ESP_LOGI(TAG, "Starting TCP server task");
    
    while (1) {
        // Create socket
        int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket, retrying in 2s...");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }
        
        // Set socket options
        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        // Configure timeout
        struct timeval timeout;
        timeout.tv_sec = 2;
        timeout.tv_usec = 0;
        setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        
        // Bind socket
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        server_addr.sin_port = htons(PORT);
        
        if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            ESP_LOGE(TAG, "Socket bind failed, retrying in 2s...");
            close(listen_sock);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }
        
        // Listen for connections
        if (listen(listen_sock, MAX_CLIENTS) < 0) {
            ESP_LOGE(TAG, "Socket listen failed, retrying in 2s...");
            close(listen_sock);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }
        
        ESP_LOGI(TAG, "TCP server started on port %d", PORT);
        ESP_LOGI(TAG, "Waiting for single character commands...");
        
        // Main server loop
        while (1) {
            // Accept client connection
            addr_len = sizeof(client_addr);
            client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
            
            if (client_sock < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // Timeout, continue listening
                    continue;
                }
                ESP_LOGE(TAG, "Accept failed: errno=%d", errno);
                break; // Break inner loop, restart server
            }
            
            ESP_LOGI(TAG, "Client connected from %s", inet_ntoa(client_addr.sin_addr));
            
            // Set receive timeout for client
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;
            setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            
            // Communication loop with client
            while (1) {
                len = recv(client_sock, &rx_char, 1, 0);
                
                if (len < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // Timeout, continue
                        continue;
                    }
                    ESP_LOGE(TAG, "Recv failed: errno=%d", errno);
                    break;
                } else if (len == 0) {
                    ESP_LOGI(TAG, "Client disconnected");
                    break;
                }
                
                // Process received character
                process_char(rx_char, client_sock);
            }
            
            // Close client socket
            close(client_sock);
            client_sock = -1;
            ESP_LOGI(TAG, "Connection closed, waiting for new connection...");
        }
        
        // Close listen socket and restart
        if (listen_sock >= 0) {
            close(listen_sock);
        }
        ESP_LOGI(TAG, "Restarting TCP server...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "Starting Quad Servo Controller Server (STA Mode)...");
    ESP_LOGI(TAG, "==================================================");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create mutex for angle data
    angle_mutex = xSemaphoreCreateMutex();
    
    // Initialize I2C and PCA9685
    i2c_master_init();
    
    if (pca9685_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCA9685!");
        return;
    }
    
    // Tắt tất cả các kênh PWM trước khi setup servo
    ESP_LOGI(TAG, "Turning off all PWM channels before setup...");
    turn_off_all_channels();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Initialize WiFi in STA mode
    wifi_init_sta();
    
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, 
                                          WIFI_CONNECTED_BIT, 
                                          pdFALSE, pdTRUE, portMAX_DELAY);
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected successfully");
    }
    
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Initialize all servos to default positions
    ESP_LOGI(TAG, "Initializing all servos to default positions:");
    
    // Tắt tất cả kênh trước khi setup từng servo
    turn_off_all_channels();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    set_servo_angle(SERVO_YAW_CHANNEL, 90);      // Yaw home (H)
    vTaskDelay(20 / portTICK_PERIOD_MS);
    set_servo_angle(SERVO_PITCH_CHANNEL, 180);   // Pitch position A
    vTaskDelay(20 / portTICK_PERIOD_MS);
    set_servo_angle(SERVO_2_CHANNEL, 60);        // Flex2 default (5)
    vTaskDelay(20 / portTICK_PERIOD_MS);
    set_servo_angle(SERVO_ROLL_CHANNEL, 0);      // Roll position X
    vTaskDelay(20 / portTICK_PERIOD_MS);
    set_servo_angle(SERVO_PITCH69_CHANNEL, 90);  // Pitch69 position U
    vTaskDelay(20 / portTICK_PERIOD_MS);
    set_servo_angle(SERVO_6_CHANNEL, 180);       // Flex1 default (1)
    
    ESP_LOGI(TAG, "  Servo 0 (Yaw): 90°");
    ESP_LOGI(TAG, "  Servo 1 (Pitch): 180°");
    ESP_LOGI(TAG, "  Servo 2 (Flex2): 60°");
    ESP_LOGI(TAG, "  Servo 4 (Roll): 0°");
    ESP_LOGI(TAG, "  Servo 5 (Pitch69): 90°");
    ESP_LOGI(TAG, "  Servo 6 (Flex1): 180°");
    
    // Start TCP server task
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    
    // Print command reference
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "Command Reference:");
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "MPU68 - Servo 0 (Yaw):");
    ESP_LOGI(TAG, "  'L' -> 10° (left >60°)");
    ESP_LOGI(TAG, "  'l' -> 50° (left >30°)");
    ESP_LOGI(TAG, "  'H' -> 90° (home)");
    ESP_LOGI(TAG, "  'r' -> 140° (right >30°)");
    ESP_LOGI(TAG, "  'R' -> 170° (right >60°)");
    
    ESP_LOGI(TAG, "MPU68 - Servo 1 (Pitch):");
    ESP_LOGI(TAG, "  'A' -> 180° (pitch -20° to 5°)");
    ESP_LOGI(TAG, "  'B' -> 150° (pitch 20.1° to 40°)");
    ESP_LOGI(TAG, "  'C' -> 120° (pitch 40.1° to 55°)");
    ESP_LOGI(TAG, "  'D' -> 90° (pitch 55.1° to 90°)");
    
    ESP_LOGI(TAG, "Flex2 - Servo 2 (GPIO34):");
    ESP_LOGI(TAG, "  '5' -> 60° (<1500)");
    ESP_LOGI(TAG, "  '6' -> 120° (1500-1600)");
    ESP_LOGI(TAG, "  '7' -> 180° (>1600)");
    
    ESP_LOGI(TAG, "MPU69 - Servo 4 (Roll):");
    ESP_LOGI(TAG, "  'X' -> 0° (Roll -160° to -180° và 160° to 180°)");
    ESP_LOGI(TAG, "  'Y' -> 45° (Roll 120° to 150°)");
    ESP_LOGI(TAG, "  'Z' -> 90° (Roll 61° to 119°)");
    ESP_LOGI(TAG, "  'W' -> 135° (Roll 21° to 60°)");
    ESP_LOGI(TAG, "  'V' -> 180° (Roll 0° to 20°)");
    
    ESP_LOGI(TAG, "MPU69 - Servo 5 (Pitch):");
    ESP_LOGI(TAG, "  'P' -> 180° (chênh > +40° so với ban đầu)");
    ESP_LOGI(TAG, "  'Q' -> 135° (chênh > +20° so với ban đầu)");
    ESP_LOGI(TAG, "  'U' -> 90° (trong khoảng ±20°)");
    ESP_LOGI(TAG, "  'T' -> 45° (chênh < -20° so với ban đầu)");
    ESP_LOGI(TAG, "  'S' -> 0° (chênh < -40° so với ban đầu)");
    
    ESP_LOGI(TAG, "Flex1 - Servo 6 (GPIO35):");
    ESP_LOGI(TAG, "  '1' -> 180° (<100)");
    ESP_LOGI(TAG, "  '2' -> 150° (100-400)");
    ESP_LOGI(TAG, "  '3' -> 120° (400-800)");
    ESP_LOGI(TAG, "  '4' -> 90° (>800)");
    
    ESP_LOGI(TAG, "Status: '?' -> Get all servo angles");
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "Quad Servo Controller READY!");
    ESP_LOGI(TAG, "==================================================");
}

