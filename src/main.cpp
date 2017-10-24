#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_deep_sleep.h"


#include "nvs_flash.h"
#include "sdkconfig.h"
#include "test.hpp"
#include "soc/gpio_struct.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

// timer defines
#define TIMER_INTR_SEL          TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP             TIMER_GROUP_0     /*!< Test on timer group 0 */
#define TIMER_DIVIDER           16               /*!< Hardware timer clock divider */
#define TIMER_SCALE             (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ          (1.4 * (TIMER_BASE_CLK / TIMER_DIVIDER) / 1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC     (0.01)   /*!< test interval for timer 0 */
#define TEST_WITHOUT_RELOAD     0   /*!< example of auto-reload mode */
#define TEST_WITH_RELOAD        1      /*!< example without auto-reload mode */

typedef struct {
    int type;                  /*!< event type */
    int group;                 /*!< timer group */
    int idx;                   /*!< timer number */
    uint64_t counter_val;      /*!< timer counter value */
} timer_event_t;

xQueueHandle timer_queue;

static void inline print_u64(uint64_t val)
{
    printf("0x%08x%08x\n", (uint32_t) (val >> 32), (uint32_t) (val));
}

// I2C pins
#define I2C_EXAMPLE_MASTER_SCL_IO           26    /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           25    /*!< gpio number for I2C master data  */
// i2c defines
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_SIZE      0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_SIZE      0   /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ          100000     /*!< I2C master clock frequency */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0         /*!< I2C ack value */
#define NACK_VAL                            0x1         /*!< I2C nack value */
#define ESP_SLAVE_ADDR                      0x28         /*!< ESP32 slave address, you can set any 7bit value */

// spi pins
#define PIN_NUM_MISO  23
#define PIN_NUM_MOSI  19
#define PIN_NUM_CLK   18
#define PIN_NUM_CS    5

// Define led pin
#define BLINK_GPIO    13

#define TAG "ExampleTask"

static EventGroupHandle_t wifi_event_group;

void sleep_task(void *pvParamter)
{
    if(esp_deep_sleep_get_wakeup_cause() == ESP_DEEP_SLEEP_WAKEUP_TIMER){
        printf("Woken up by timer\n");
    }
    else {
        printf("Woken up by something else\n");
    }

    const int wakeup_time_sec = 5;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_deep_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);
    printf("Entering deep sleep\n");
    esp_deep_sleep_start();
}

static esp_err_t i2c_example_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void i2c_task(void *pvParameter)
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config((i2c_port_t)i2c_master_port, &conf);
    i2c_driver_install((i2c_port_t)i2c_master_port, conf.mode, I2C_EXAMPLE_MASTER_RX_BUF_SIZE, I2C_EXAMPLE_MASTER_TX_BUF_SIZE, 0);

    uint8_t data_rd[20];

    while(1) {
        i2c_example_master_read_slave(I2C_EXAMPLE_MASTER_NUM, data_rd, 10);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void spi_task(void *pvParameter)
{
    esp_err_t ret;
    spi_device_handle_t spi;

    spi_bus_config_t buscfg;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    spi_device_interface_config_t devcfg;
    devcfg.clock_speed_hz = 10*1000*1000;
    devcfg.mode = 0;
    devcfg.spics_io_num = PIN_NUM_CS;
    devcfg.queue_size = 8;

    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);

    uint8_t data[8] = {1,2,3,4,5,6,7,8};
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to

    while(1) {
        ret=spi_device_transmit(spi, &t);  //Transmit!
        assert(ret==ESP_OK);            //Should have had no issues.
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void blink_task(void *pvParameter)
{
    gpio_pad_select_gpio((gpio_num_t)BLINK_GPIO);
    // Set the GPIO as a push/pull output
    gpio_set_direction((gpio_num_t)BLINK_GPIO, (gpio_mode_t)GPIO_MODE_OUTPUT);
    while(1) {
        // Blink off (output low)
        gpio_set_level((gpio_num_t)BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // Blink on (output high)
        gpio_set_level((gpio_num_t)BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    timer_event_t evt;
    if((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        //Timer0 is an example that doesn't reload counter value
        TIMERG0.hw_timer[timer_idx].update = 1;
        // We don't call a API here because they are not declared with IRAM_ATTR.
        // If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
        // we can alloc this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
        TIMERG0.int_clr_timers.t0 = 1;
        uint64_t timer_val = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;
        //Post an event to out example task
        evt.type = TEST_WITHOUT_RELOAD;
        evt.group = 0;
        evt.idx = timer_idx;
        evt.counter_val = timer_val;
        xQueueSendFromISR(timer_queue, &evt, NULL);
        //For a timer that will not reload, we need to set the next alarm value each time.
        timer_val += (uint64_t) (TIMER_INTERVAL0_SEC * (TIMER_BASE_CLK / TIMERG0.hw_timer[timer_idx].config.divider));
        //Fine adjust
        timer_val -= TIMER_FINE_ADJ;
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_val >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_val;
        //After set alarm, we set alarm_en bit if we want to enable alarm again.
        TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;
    }
}

static void example_tg0_timer0_init()
{
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    timer_group_t timer_group = TIMER_GROUP_0;
    timer_idx_t timer_idx = TIMER_0;
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 0;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, TIMER_INTERVAL0_SEC * TIMER_SCALE - TIMER_FINE_ADJ);
    /*Enable timer interrupt*/
    timer_enable_intr(timer_group, timer_idx);
    /*Set ISR handler*/
    timer_isr_register(timer_group, timer_idx, timer_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    /*Start timer counter*/
    timer_start(timer_group, timer_idx);
}

static void timer_example_evt_task(void *arg)
{
    example_tg0_timer0_init();

    int counter = 0;

    timer_event_t evt;

    while(1) {
        // wait for queue to be ready
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        // feedback
        if(counter++ == 100) {
            ESP_LOGI(TAG, "Tick");
            counter = 0;
        }
    }
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, BIT0);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, BIT0);
            break;
        default:
            break;
    }
    return ESP_OK;
}

bool wifi_connect_to_ap(char* SSID, char* PASS, int timeout) {
    tcpip_adapter_init();

    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, SSID);
    if(PASS != NULL) {
      strcpy((char*)wifi_config.sta.password, PASS);
    }
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );

    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_start() );

    xEventGroupWaitBits(wifi_event_group, BIT0, false, true, timeout);
    return (bool) (xEventGroupGetBits(wifi_event_group) & 1);
}

bool wifi_set_up_ap(char* SSID, char* PASS) {
    tcpip_adapter_init();

    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );

    wifi_config_t conf;
    strcpy((char*)conf.ap.ssid, SSID);
    conf.ap.ssid_len = strlen(SSID);
    if(PASS != NULL) {
      strcpy((char*)conf.ap.password, PASS);
      conf.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }
    else {
      *conf.ap.password = 0;
      conf.ap.authmode = WIFI_AUTH_OPEN;
    }
    conf.ap.channel = 5;
    conf.ap.ssid_hidden = 0;
    conf.ap.max_connection = 4;
    conf.ap.beacon_interval = 100;
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", conf.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_AP, &conf) );

    ESP_ERROR_CHECK( esp_wifi_start() );
    return true;
}

extern "C" void app_main(void)
{
    test newTest;

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("\n\n\nBooting complete!\n");
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024), (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    nvs_flash_init();

    //wifi_set_up_ap((char*)"Allyouare", (char*)"Meulen-2017");
    wifi_connect_to_ap((char*)"eduroam", (char*)"Sander1994", 500);
    esp_wifi_stop();

    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    //xTaskCreate(&spi_task, "spi_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    //xTaskCreate(&i2c_task, "i2c_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    //xTaskCreate(&sleep_task, "sleep_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
}
