/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_rom_md5.h" // to get hash function
#include "wifi.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define CAMERA_ESP_WIFI_SSID1      setup_wifi_details
#define CAMERA_ESP_WIFI_PASS1      setup_wifi_details

#if __has_include("wifi_details.h")
#undef CAMERA_ESP_WIFI_SSID1
#undef CAMERA_ESP_WIFI_PASS1

#include "wifi_details.h"
#endif

#define CAMERA_ESP_MAXIMUM_RETRY  10

char wifi_ssid[33]  = { 0 };

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi";

static char g_hostname[32] = "esp_idf";
static bool connected = true;
static bool do_reconnect = false;
static int s_retry_num = 0;

static unsigned short get_id()
{
    md5_context_t context;
    unsigned char digest[16];
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    esp_rom_md5_init(&context);
    esp_rom_md5_update(&context, mac, 6);
    esp_rom_md5_final(digest, &context);
    return (digest[14] << 8) | digest[15];
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        #if 0
        char name[42];
        snprintf(name, sizeof(name), "%s_%u", g_hostname, get_id());
        //esp_err_t err = tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, name);
        esp_err_t err = esp_netif_set_hostname()
        if (err != ESP_OK)
        {
            ESP_LOGI(TAG, "setting hostname err %d", err);
        }
        #endif
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (!connected || s_retry_num < CAMERA_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail %d", connected);
        connected = false;
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "wifi connected");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
    {
        ESP_LOGI(TAG, "wifi scan done");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        connected = true;
    }
}

static esp_err_t wifi_perform_scan()
{
    for (;;)
    {
        wifi_scan_config_t scan_config{};

        esp_err_t err = esp_wifi_scan_start(&scan_config, true);
        if (err != ESP_OK)
        {
            ESP_LOGI(TAG, "Failed to start wifi scan %s %d",  esp_err_to_name(err), err);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        uint16_t scan_ap_num;
        esp_wifi_scan_get_ap_num(&scan_ap_num);
        if (scan_ap_num == 0) {
            ESP_LOGI(TAG, "No matching APs found in scan");
            esp_wifi_scan_stop();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        wifi_ap_record_t *ap_list_buffer = (wifi_ap_record_t *)malloc(scan_ap_num * sizeof(wifi_ap_record_t));
        if (ap_list_buffer == NULL) {
            ESP_LOGE(TAG, "Failed to malloc buffer to print scan results");
            return ESP_FAIL;
        }

        int strongest = -1;
        int8_t max_rssi;
        if (esp_wifi_scan_get_ap_records(&scan_ap_num, ap_list_buffer) == ESP_OK) {
            for (uint16_t i = 0; i < scan_ap_num; i++) {
                const char *ap_ssid = (const char *)ap_list_buffer[i].ssid;
                ESP_LOGI(TAG, "[%s][rssi=%d]""%s", ap_ssid, ap_list_buffer[i].rssi,
                            ap_list_buffer[i].ftm_responder ? "[FTM Responder]" : "");
                int selected = -1;
#if defined(CAMERA_ESP_WIFI_SSID1)
                if (strcmp(ap_ssid, CAMERA_ESP_WIFI_SSID1) == 0)
                {
                    selected = 1;
                }
#endif
#if defined(CAMERA_ESP_WIFI_SSID2)
                if (strcmp(ap_ssid, CAMERA_ESP_WIFI_SSID2) == 0)
                {
                    selected = 2;
                }
#endif
                if (selected < 0)
                {
                    continue;
                }

                if (strongest < 0)
                {
                    strongest = i;
                    max_rssi = ap_list_buffer[i].rssi;
                }
                else if (ap_list_buffer[i].rssi > max_rssi)
                {
                    strongest = i;
                    max_rssi = ap_list_buffer[i].rssi;
                }
            }
        }

        if (strongest < 0) {
            ESP_LOGI(TAG, "Did not find recognised AP");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "sta scan done, found %d, strongest %d %s", scan_ap_num, strongest, ap_list_buffer[strongest].ssid);
        strlcpy(wifi_ssid, (char *)ap_list_buffer[strongest].ssid, sizeof(wifi_ssid));
        free(ap_list_buffer);

        if (scan_ap_num > 0)
        {
            return ESP_OK;
        }
    }

    return ESP_FAIL;
}

static esp_event_handler_instance_t instance_any_id;
static esp_event_handler_instance_t instance_got_ip;
static esp_netif_t *netif;

esp_netif_t *wifi_get_netif()
{
    return netif;
}

void wifi_reconnect() 
{
    if (!connected && do_reconnect)
    {
        ESP_LOGI(TAG, "try to reconnect to wifi");
        esp_wifi_connect();
    }
}

void wifi_init_sta(const char *hostname, bool with_bluetooth)
{
    if (hostname != nullptr)
    {
        strlcpy(g_hostname, hostname, sizeof(g_hostname));
    }
    
    s_wifi_event_group = xEventGroupCreate();

    //ESP_ERROR_CHECK(esp_netif_init());

    //ESP_ERROR_CHECK(esp_event_loop_create_default());
    netif = esp_netif_create_default_wifi_sta();
    char name[42];
    snprintf(name, sizeof(name), "%s_%u", g_hostname, get_id());
    esp_err_t err = esp_netif_set_hostname(netif, name);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "setting hostname err %d", err);
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    err = esp_wifi_restore();
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "wifi restore err %d", err);
    }
    
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

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

    for (;;)
    {
        ESP_LOGI(TAG, "esp_wifi_start for scan.");
        ESP_ERROR_CHECK(esp_wifi_start() );
        wifi_perform_scan();
        ESP_LOGI(TAG, "esp_wifi_stop after scan.");
        ESP_ERROR_CHECK(esp_wifi_stop() );

        wifi_config_t wifi_config{};
        memcpy(wifi_config.sta.ssid, wifi_ssid, strlen(wifi_ssid) + 1);
        if (strcmp(wifi_ssid, CAMERA_ESP_WIFI_SSID1) == 0)
        {
            memcpy(wifi_config.sta.password, CAMERA_ESP_WIFI_PASS1, sizeof(CAMERA_ESP_WIFI_PASS1));
        }
        else
        {
            memcpy(wifi_config.sta.password, CAMERA_ESP_WIFI_PASS2, sizeof(CAMERA_ESP_WIFI_PASS2));
        }
        /* Setting a password implies station will connect to all security modes including WEP/WPA.
            * However these modes are deprecated and not advisable to be used. Incase your Access point
            * doesn't support WPA2, these mode can be enabled by commenting below line */
        if (strlen((char *)wifi_config.sta.password) == 0)
        {
            wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
        }
        else
        {
            wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        }

        wifi_config.sta.pmf_cfg.capable = true;
        wifi_config.sta.pmf_cfg.required = false;

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        ESP_ERROR_CHECK(esp_wifi_start() );

        if (!with_bluetooth)
        {
            ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE) );
        }

        ESP_LOGI(TAG, "wifi_init_sta finished.");

        /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
        * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                pdFALSE,
                pdFALSE,
                portMAX_DELAY);

        /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
        * happened. */
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                    wifi_ssid, wifi_config.sta.password);
            break;
        } else if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                    wifi_ssid, wifi_config.sta.password);
            s_retry_num = 0;
        } else {
            ESP_LOGE(TAG, "UNEXPECTED EVENT");
        }

        do_reconnect = true;
    }
}

void wifi_cleanup()
{
    connected = false;
    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

#if 0
void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
}
#endif