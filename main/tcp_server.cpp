
extern "C"{
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include <stdio.h>
#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "driver/ledc.h"
#include "driver/dac.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
}
#include <map>
#define EXAMPLE_ESP_WIFI_SSID      "ESP32_AP"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_MAX_STA_CONN       1
#define KEEPALIVE_IDLE              CONFIG_EXAMPLE_KEEPALIVE_IDLE
#define KEEPALIVE_INTERVAL          CONFIG_EXAMPLE_KEEPALIVE_INTERVAL
#define KEEPALIVE_COUNT             CONFIG_EXAMPLE_KEEPALIVE_COUNT
#define PORT                        12345
// 핀 및 LED 설정
#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_0    LEDC_CHANNEL_0
#define LEDC_CHANNEL_1    LEDC_CHANNEL_1
#define LEDC_CHANNEL_2    LEDC_CHANNEL_2
#define LEDC_DUTY_RES     LEDC_TIMER_8_BIT

#define RED_PIN   12
#define GREEN_PIN 13
#define BLUE_PIN  14
#define DAC_CHANNEL DAC_CHANNEL_1

#define AUDIO_FILE_PATH "/spiffs/music.wav"

static const char *TAG = "softAP_example";
void rgb_set_color(uint8_t r, uint8_t g, uint8_t b);
void rgb_set_color_by_time(uint8_t hour);

static FILE *audioFile = NULL;
static bool wav_received = false;
static bool fft_received = false;
static bool playback_started = false;
static float* analyzedData = NULL;
static size_t analyzedDataLength = 0;

void audio_playback_task(void* arg) {
    // WAV 파일 열기
    audioFile = fopen(AUDIO_FILE_PATH, "rb");
    if (!audioFile) {
        ESP_LOGE(TAG, "재생할 파일 열기 실패");
        vTaskDelete(NULL);
    }

    uint8_t buf[512];
    size_t len = 0;

    // WAV 파일 헤더 건너뛰기
    fseek(audioFile, 44, SEEK_SET);  // WAV 파일 헤더는 44바이트

    while ((len = fread(buf, 1, sizeof(buf), audioFile)) > 0) {
        for (size_t i = 0; i < len; i++) {
            // 16비트 PCM 데이터를 8비트로 변환하여 DAC 출력
            uint8_t audio_sample = buf[i] / 2;  // 8비트로 축소 (간단한 변환 예시)
            dac_output_voltage(DAC_CHANNEL, audio_sample);  // DAC1(GPIO 25) 핀으로 출력
        }
    }

    fclose(audioFile);
    ESP_LOGI(TAG, "WAV 재생 완료");

    // 메모리 해제
    if (analyzedData) {
        free(analyzedData);
        analyzedData = NULL;
    }

    vTaskDelete(NULL);
}

void led_control_task(void* arg) {
    while (1) {
        if (analyzedData != NULL) {
            for (int i = 0; i < analyzedDataLength; i++) {
                float amp = analyzedData[i];  // 0.0 ~ 1.0 범위라고 가정
                uint8_t r = amp * 255;
                uint8_t g = 255 - r;
                uint8_t b = (amp < 0.5) ? r : 255 - r;
                rgb_set_color(r, g, b);
                vTaskDelay(pdMS_TO_TICKS(100));  // 예: 100ms 간격
            }
        } else {
            ESP_LOGW(TAG, "FFT 데이터가 없습니다.");
            vTaskDelay(pdMS_TO_TICKS(1000));  // 데이터 없으면 1초 대기
        }
    }
}

static void handle_incoming_command(const int sock) {
    int len;
    char rx_buffer[128];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // 문자열처럼 보기 위해 null-terminate

            uint8_t cmd = rx_buffer[0];

            switch (cmd) {
                case 0x01: { // RGB 색상 직접 설정
                    if (len >= 4) {
                        uint8_t r = rx_buffer[1];
                        uint8_t g = rx_buffer[2];
                        uint8_t b = rx_buffer[3];
                        rgb_set_color(r, g, b);
                    } else {
                        ESP_LOGW(TAG, "RGB 색상 명령 길이 부족");
                    }
                    break;
                }

                case 0x02: { // 시간에 맞는 RGB 색상 설정
                    if (len >= 2) {
                        uint8_t hour = rx_buffer[1];
                        rgb_set_color_by_time(hour);
                    } else {
                        ESP_LOGW(TAG, "시간 설정 명령 길이 부족");
                    }
                    break;
                }
                case 0x03: { // WAV 파일 수신
                    ESP_LOGI(TAG, "WAV 파일 수신 시작");

                    // (1) 파일 크기 받기 (4바이트, uint32_t)
                    if (len < 5) {
                        ESP_LOGW(TAG, "파일 크기 정보 부족");
                        break;
                    }
                    uint32_t file_size = *(uint32_t*)&rx_buffer[1];

                    audioFile = fopen(AUDIO_FILE_PATH, "wb");
                    if (!audioFile) {
                        ESP_LOGE(TAG, "WAV 파일 열기 실패");
                        break;
                    }

                    int received = 0;
                    while (received < file_size) {
                        int chunk = recv(sock, rx_buffer, sizeof(rx_buffer), 0);
                        if (chunk <= 0) break;
                        fwrite(rx_buffer, 1, chunk, audioFile);
                        received += chunk;
                    }

                    fclose(audioFile);
                    wav_received = true;
                    ESP_LOGI(TAG, "WAV 파일 수신 완료");
                    break;
                }

                case 0x04: { // FFT 데이터 수신
                    ESP_LOGI(TAG, "FFT 데이터 수신 시작");

                    // (1) 데이터 개수 받기 (4바이트, uint32_t)
                    if (len < 5) {
                        ESP_LOGW(TAG, "데이터 개수 정보 부족");
                        break;
                    }
                    analyzedDataLength = *(uint32_t*)&rx_buffer[1];

                    analyzedData = (float*)malloc(sizeof(float) * analyzedDataLength);
                    if (!analyzedData) {
                        ESP_LOGE(TAG, "메모리 할당 실패");
                        break;
                    }

                    int received = 0;
                    uint8_t* buffer_ptr = (uint8_t*)analyzedData;
                    int total_bytes = sizeof(float) * analyzedDataLength;
                    while (received < total_bytes) {
                        int chunk = recv(sock, rx_buffer, sizeof(rx_buffer), 0);
                        if (chunk <= 0) break;
                        memcpy(buffer_ptr + received, rx_buffer, chunk);
                        received += chunk;
                    }

                    fft_received = true;
                    ESP_LOGI(TAG, "FFT 데이터 수신 완료");
                    break;
                }

                default:
                    ESP_LOGW(TAG, "알 수 없는 명령");
                    break;
            }

            // WAV + FFT 데이터 수신 완료 후 재생 및 LED 제어 시작
            if (wav_received && fft_received && !playback_started) {
                ESP_LOGI(TAG, "WAV + FFT 수신 완료 → 재생 및 LED PWM 시작");
                playback_started = true;

                // WAV 재생 Task
                xTaskCreate(&audio_playback_task, "audio_play", 4096, NULL, 5, NULL);

                // LED PWM 제어 Task
                xTaskCreate(&led_control_task, "led_pwm", 2048, NULL, 5, NULL);
            }
        }
    } while (len > 0);
}


static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

#ifdef CONFIG_EXAMPLE_IPV4
    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
#ifdef CONFIG_EXAMPLE_IPV4
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
#ifdef CONFIG_EXAMPLE_IPV6
        if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        handle_incoming_command(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void wifi_init_softap(void)
{
    esp_netif_create_default_wifi_ap();  // AP 인터페이스 생성

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.ap.ssid, EXAMPLE_ESP_WIFI_SSID);
    strcpy((char *)wifi_config.ap.password, EXAMPLE_ESP_WIFI_PASS);
    wifi_config.ap.ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID);
    wifi_config.ap.max_connection = EXAMPLE_MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));  // AP 모드로 설정
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));  // 설정 적용
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi SoftAP initialized. SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

void rgb_led_init() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t channels[3] = {
        {RED_PIN, LEDC_MODE, LEDC_CHANNEL_0, LEDC_INTR_DISABLE, LEDC_TIMER, 0, 0},
        {GREEN_PIN, LEDC_MODE, LEDC_CHANNEL_1, LEDC_INTR_DISABLE, LEDC_TIMER, 0, 0},
        {BLUE_PIN, LEDC_MODE, LEDC_CHANNEL_2, LEDC_INTR_DISABLE, LEDC_TIMER, 0, 0}
    };

    for (int i = 0; i < 3; i++) {
        ledc_channel_config(&channels[i]);
    }
}

void rgb_set_color(uint8_t r, uint8_t g, uint8_t b) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, r);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, g);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, b);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);
}
void rgb_set_color_by_time(uint8_t hour) {
    uint8_t r = 255, g = 255, b = 255;

    if (hour >= 6 && hour < 9) {
        r = 100; g = 150; b = 255;
    } else if (hour >= 9 && hour < 17) {
        r = 120; g = 180; b = 255;
    } else if (hour >= 17 && hour < 19) {
        r = 255; g = 200; b = 100;
    } else if (hour >= 19 && hour < 22) {
        r = 255; g = 160; b = 80;
    } else {
        r = 200; g = 100; b = 30;
    }

    rgb_set_color(r, g, b);
}


extern "C" void app_main(void)

{
    rgb_led_init();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_softap();  // SoftAP 설정 호출

    // 여기에 tcp_server_task 호출하면 됨
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void *)AF_INET, 5, NULL);
}
