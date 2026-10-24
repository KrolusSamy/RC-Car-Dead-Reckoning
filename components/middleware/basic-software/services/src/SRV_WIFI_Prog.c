/* Wi-Fi station service. APP owns the system LED and queries link state here. */
#include "SRV_WIFI_Interface.h"
#include "SRV_WIFI_Cfg.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "SRV_WIFI";
static int retry_count;
static volatile bool connected;
static bool initialized;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        connected = false;
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) ESP_LOGE(TAG, "Initial Wi-Fi connect request failed: %s", esp_err_to_name(err));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        connected = false;
        if (retry_count < SRV_WIFI_MAX_RETRY) retry_count++;
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) ESP_LOGW(TAG, "Wi-Fi reconnect request failed (transient): %s", esp_err_to_name(err));
        else ESP_LOGW(TAG, "Wi-Fi disconnected; reconnecting (attempt counter %d)", retry_count);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        retry_count = 0;
        connected = true;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

bool SRV_WIFI_IsConnected(void)
{
    return connected;
}

ErrorState_TypeDef SRV_WIFI_Init(void)
{
    if (initialized) return FUNCTION_OK;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) err = nvs_flash_init();
    }
    if (err != ESP_OK) { ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(err)); return FUNCTION_ERROR; }
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err)); return FUNCTION_ERROR; }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { ESP_LOGE(TAG, "Event loop initialization failed: %s", esp_err_to_name(err)); return FUNCTION_ERROR; }
    if (esp_netif_create_default_wifi_sta() == NULL) { ESP_LOGE(TAG, "Could not create station network interface"); return FUNCTION_ERROR; }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err)); return FUNCTION_ERROR; }
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Wi-Fi event registration failed: %s", esp_err_to_name(err)); return FUNCTION_ERROR; }
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL);
    if (err != ESP_OK) { ESP_LOGE(TAG, "IP event registration failed: %s", esp_err_to_name(err)); return FUNCTION_ERROR; }

    wifi_config_t wifi_config = { .sta = {
        .ssid = SRV_WIFI_SSID,
        .password = SRV_WIFI_PASS,
        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
    }};
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) { ESP_LOGE(TAG, "Wi-Fi start/configuration failed: %s", esp_err_to_name(err)); return FUNCTION_ERROR; }
    initialized = true;
    ESP_LOGI(TAG, "Wi-Fi station initialized; connecting to configured hotspot");
    return FUNCTION_OK;
}

