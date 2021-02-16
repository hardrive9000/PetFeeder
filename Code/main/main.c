#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/netdb.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// wifi bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define HTTP_PORT 80 // http port number
#define SERVO_PIN GPIO_NUM_18 // servo pin
#define BUTTON_PIN GPIO_NUM_19 // push-button pin
#define LED_PIN GPIO_NUM_5 // led pin
#define FEED_RATE 1333 // servo feed ratio
#define FEED_REVERSAL 1722 // servo feed reversal ratio
#define FEED_STOP 1500 // servo stop

// http headers and web pages
const static char http_html_hdr[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";
const static char http_png_hdr[] = "HTTP/1.1 200 OK\nContent-type: image/png\n\n";
const static char http_off_hml[] = "<meta content=\"width=device-width,initial-scale=1\"name=viewport><style>div{width:230px;height:300px;position:absolute;top:0;bottom:0;left:0;right:0;margin:auto}</style><div><h1 align=center>ALIMENTAR</h1><a href=on.html><img src=on.png></a></div>";
const static char http_on_hml[] = "<meta content=\"width=device-width,initial-scale=1\"name=viewport><style>div{width:230px;height:300px;position:absolute;top:0;bottom:0;left:0;right:0;margin:auto}</style><div><h1 align=center>HECHO</h1><a href=off.html><img src=off.png></a></div>";

// embedded binary data (on/off images)
extern const uint8_t on_png_start[] asm("_binary_on_png_start");
extern const uint8_t on_png_end[]   asm("_binary_on_png_end");
extern const uint8_t off_png_start[] asm("_binary_off_png_start");
extern const uint8_t off_png_end[]   asm("_binary_off_png_end");

// event group for inter-task communication
static EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;

static bool busy = false;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
         .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void servo_init()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN); // set GPIO 18 as PWM0A, to which servo is connected

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0; // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0; // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void gpio_pins_init()
{
    gpio_config_t btn_config, led_config;

    btn_config.mode = GPIO_MODE_INPUT; // set as input
    btn_config.pin_bit_mask = (1 << BUTTON_PIN); // bitmask
    btn_config.pull_up_en = GPIO_PULLUP_ENABLE; // enable pullup
    btn_config.pull_down_en = GPIO_PULLDOWN_DISABLE; // disable pulldown
    gpio_config(&btn_config);

    led_config.mode = GPIO_MODE_OUTPUT; // set as output
    led_config.pin_bit_mask = (1 << LED_PIN); // bitmask
    led_config.pull_up_en = GPIO_PULLUP_DISABLE; // disable pullup
    led_config.pull_down_en = GPIO_PULLDOWN_DISABLE; // disable pulldown
    gpio_config(&led_config);

    gpio_set_level(LED_PIN, 0); // led off
}

void feed()
{
    gpio_set_level(LED_PIN, 1); // led on

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, FEED_RATE); // start feeding for 2s
    vTaskDelay(2000 / portTICK_RATE_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, FEED_REVERSAL); // reverse direction for 0.5s to prevent clogging
    vTaskDelay(500 / portTICK_RATE_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, FEED_RATE); // continue feeding for 2s
    vTaskDelay(2000 / portTICK_RATE_MS);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, FEED_STOP); // stop feeding

    gpio_set_level(LED_PIN, 0); // led off
}

void http_server_netconn_serve(struct netconn *conn) {

    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;

    err = netconn_recv(conn, &inbuf);

    if (err == ERR_OK) {

        netbuf_data(inbuf, (void**)&buf, &buflen);

        // extract the first line, with the request
        char *first_line = strtok(buf, "\n");

        if(first_line) {

            // default page
            if(strstr(first_line, "GET / ")) {
                netconn_write(conn, http_html_hdr, sizeof(http_html_hdr) - 1, NETCONN_NOCOPY);
                netconn_write(conn, http_off_hml, sizeof(http_off_hml) - 1, NETCONN_NOCOPY);
            }

            // ON page
            else if(strstr(first_line, "GET /on.html ")) {
                netconn_write(conn, http_html_hdr, sizeof(http_html_hdr) - 1, NETCONN_NOCOPY);
                netconn_write(conn, http_on_hml, sizeof(http_on_hml) - 1, NETCONN_NOCOPY);

                if (busy == false) {
                    busy = true;
                    feed();
                    busy = false;
                }
            }

            // OFF page
            else if(strstr(first_line, "GET /off.html ")) {
                netconn_write(conn, http_html_hdr, sizeof(http_html_hdr) - 1, NETCONN_NOCOPY);
                netconn_write(conn, http_off_hml, sizeof(http_off_hml) - 1, NETCONN_NOCOPY);
            }

            // ON image
            else if(strstr(first_line, "GET /on.png ")) {
                netconn_write(conn, http_png_hdr, sizeof(http_png_hdr) - 1, NETCONN_NOCOPY);
                netconn_write(conn, on_png_start, on_png_end - on_png_start, NETCONN_NOCOPY);
            }

            // OFF image
            else if(strstr(first_line, "GET /off.png ")) {
                netconn_write(conn, http_png_hdr, sizeof(http_png_hdr) - 1, NETCONN_NOCOPY);
                netconn_write(conn, off_png_start, off_png_end - off_png_start, NETCONN_NOCOPY);
            }
        }
    }

    // close the connection and free the buffer
    netconn_close(conn);
    netbuf_delete(inbuf);
}

void button_task(void *pvParameters)
{
    for (;;) {

        if ((gpio_get_level(BUTTON_PIN) == 0) && (busy == false)) {
            busy = true;
            feed();
            busy = false;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void http_server(void *pvParameters) {

    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, HTTP_PORT);
    netconn_listen(conn);

    do {
        err = netconn_accept(conn, &newconn);

        if (err == ERR_OK) {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
        vTaskDelay(1); //allows task to be pre-empted
    } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // disable the default wifi logging
    esp_log_level_set("wifi", ESP_LOG_NONE);
    wifi_init_sta();

    servo_init(); // init MCPWM servo pin
    gpio_pins_init(); // init GPIO for push-button and led

    // start the http server task
    xTaskCreate(&http_server, "http_server", 2048, NULL, 5, NULL);

    // start the push-button polling task
    xTaskCreate(&button_task, "button", 2048, NULL, 5, NULL);
}
