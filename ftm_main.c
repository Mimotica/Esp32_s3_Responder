#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_timer.h"

#define EXAMPLE_AP_SSID    "ESP_FTM_AP"
#define EXAMPLE_AP_PASS    "12345678"
#define EXAMPLE_AP_CHANNEL 1

// we're using µs resolution, bursts every 1 s = 1 000 000 µs
#define EXPECTED_INTERVAL_US 1000000ULL
#define BURST_TIMEOUT_US     200000ULL   // 200 ms without packets ends a burst

typedef uint64_t TIME_T;
static const char *TAG = "FTM_RESPONDER";

// high‑res GPTimer @1 MHz → 1 µs ticks
static gptimer_handle_t hr_timer_handle;

// burst‑to‑burst buffer
#define CIRC_BUFFER_SIZE 10
static TIME_T burst_intervals[CIRC_BUFFER_SIZE];
static int     bi_idx        = 0;
static int     bi_count      = 0;
static TIME_T  avg_burst_ivl = 0;

// session state
static bool    session_active = false;
static TIME_T  last_burst_ts = 0;

// compute and log running average of burst intervals
static void update_burst_interval(TIME_T ivl)
{
    burst_intervals[bi_idx] = ivl;
    bi_idx = (bi_idx + 1) % CIRC_BUFFER_SIZE;
    if (bi_count < CIRC_BUFFER_SIZE) bi_count++;

    TIME_T sum = 0;
    for (int i = 0; i < bi_count; i++) {
        sum += burst_intervals[i];
    }
    avg_burst_ivl = sum / bi_count;

    int64_t inst_offset = (int64_t)ivl - (int64_t)EXPECTED_INTERVAL_US;
    int64_t avg_offset  = (int64_t)avg_burst_ivl - (int64_t)EXPECTED_INTERVAL_US;

    ESP_LOGI(TAG, "Burst Δ = %" PRIu64 " us, Inst. Offset = %" PRId64 " us, Avg Offset = %" PRId64 " us",
             ivl, inst_offset, avg_offset);
}

// promiscuous callback: catch *all* mgmt frames and look at their arrival time
static void promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT) return;
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buf;
    uint8_t hdr0 = pkt->payload[0];
    uint8_t subtype = (hdr0 >> 4) & 0x0F;
    const uint8_t ACTION_SUBTYPE = 0x0D;
    if (subtype != ACTION_SUBTYPE) return;

    // timestamp arrival
    TIME_T now;
    ESP_ERROR_CHECK(gptimer_get_raw_count(hr_timer_handle, &now));

    if (!session_active) {
        // first frame of a burst
        ESP_LOGI(TAG, "FTM burst start ts = %" PRIu64 " us", now);

        if (last_burst_ts != 0) {
            update_burst_interval(now - last_burst_ts);
        }
        last_burst_ts = now;
        session_active = true;
    }
    // otherwise, ignore in‑burst frames
}

// monitor task to detect end of burst (no frames for BURST_TIMEOUT_US)
static void session_monitor_task(void *pv)
{
    while (1) {
        if (session_active) {
            uint64_t now_us = esp_timer_get_time();
            if (now_us - last_burst_ts >= BURST_TIMEOUT_US) {
                // burst ended
                ESP_LOGI(TAG, "Burst ended");
                session_active = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    wifi_config_t ap_cfg = { 0 };
    strlcpy((char*)ap_cfg.ap.ssid,    EXAMPLE_AP_SSID, sizeof(ap_cfg.ap.ssid));
    strlcpy((char*)ap_cfg.ap.password,EXAMPLE_AP_PASS,sizeof(ap_cfg.ap.password));
    ap_cfg.ap.ssid_len       = strlen(EXAMPLE_AP_SSID);
    ap_cfg.ap.channel        = EXAMPLE_AP_CHANNEL;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode       = WIFI_AUTH_WPA_WPA2_PSK;
    ap_cfg.ap.ftm_responder  = true;
    // allow non‑PMF STAs
    ap_cfg.ap.pmf_cfg.capable  = true;
    ap_cfg.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "SoftAP up: %s ch %d", EXAMPLE_AP_SSID, EXAMPLE_AP_CHANNEL);

    // promiscuous sniff only mgmt frames
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    wifi_promiscuous_filter_t filt = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filt));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(promisc_cb));
}

void app_main(void)
{
    wifi_init_softap();

    // setup GPTimer @1 MHz
    gptimer_config_t tcfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &hr_timer_handle));
    ESP_ERROR_CHECK(gptimer_enable(hr_timer_handle));
    ESP_ERROR_CHECK(gptimer_start(hr_timer_handle));

    // launch burst‑end detector
    xTaskCreate(session_monitor_task, "session_mon", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "Responder ready; bursts expected every %llu us", EXPECTED_INTERVAL_US);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
