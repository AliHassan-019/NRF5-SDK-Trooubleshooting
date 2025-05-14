/**
 * @file    main.c
 * @brief   Read two NTC channels via SAADC and send readings over BLE using Nordic UART Service.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrfx_saadc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_nus.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "nrf_ble_gatt.h"
#include "ble_conn_params.h"

// GPIO / SAADC definitions
#define LED_PIN             24                // P0.24
#define SR_RESET_PIN        13                // P0.13
#define NTC_EN_PIN          29                // P0.29
#define NTC1_AIN            NRF_SAADC_INPUT_AIN6  // P0.30
#define NTC2_AIN            NRF_SAADC_INPUT_AIN7  // P0.31
#define SAADC_BUFFER_SIZE   2
#define SAADC_SAMPLE_INTERVAL_MS 1000         // 1s
#define APP_BLE_CONN_CFG_TAG    1
#define DEVICE_NAME             "NTC_BLE"      // BLE device name

// BLE parameters
#define MIN_CONN_INTERVAL       MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL       MSEC_TO_UNITS(75, UNIT_1_25_MS)
#define SLAVE_LATENCY           0
#define CONN_SUP_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)
#define APP_ADV_INTERVAL        64             // 40 ms
#define APP_ADV_DURATION        18000          // 180 seconds

// Module instances
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);            
NRF_BLE_GATT_DEF(m_gatt);                                    
BLE_ADVERTISING_DEF(m_advertising);                          

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

// SAADC buffers
static nrf_saadc_value_t saadc_buf1[SAADC_BUFFER_SIZE];
static nrf_saadc_value_t saadc_buf2[SAADC_BUFFER_SIZE];

// Forward declarations
static void ble_stack_init(void);
static void gap_params_init(void);
static void gatt_init(void);
static void services_init(void);
static void advertising_init(void);
static void conn_params_init(void);
static void advertising_start(void);
static void saadc_init(void);
static void idle_state_handle(void);

//=== Logging & Timers ===
static void log_init(void)
{
    ret_code_t err = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void timers_init(void)
{
    ret_code_t err = app_timer_init();
    APP_ERROR_CHECK(err);
}

//=== BLE Event Handlers ===
static void on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            NRF_LOG_INFO("Connected");
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            NRF_LOG_INFO("Disconnected");
            advertising_start();
            break;

        default:
            break;
    }
}

NRF_SDH_BLE_OBSERVER(m_ble_obs, APP_BLE_CONN_CFG_TAG, on_ble_evt, NULL);

//=== GAP & GATT ===
static void gap_params_init(void)
{
    ret_code_t err;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err = sd_ble_gap_device_name_set(&sec_mode,
                                     (const uint8_t *)DEVICE_NAME,
                                     strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err);

    ble_gap_conn_params_t gap_conn_params = {
        .min_conn_interval = MIN_CONN_INTERVAL,
        .max_conn_interval = MAX_CONN_INTERVAL,
        .slave_latency     = SLAVE_LATENCY,
        .conn_sup_timeout  = CONN_SUP_TIMEOUT
    };
    err = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err);
}

static void gatt_init(void)
{
    ret_code_t err = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err);
}

//=== NUS Service ===
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    // We don't expect data from central in this example
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_INFO("Received data from central");
    }
}

static void services_init(void)
{
    ret_code_t         err;
    ble_nus_init_t     nus_init;

    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;

    err = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err);
}

//=== Advertising ===
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    if (ble_adv_evt == BLE_ADV_EVT_FAST)
    {
        NRF_LOG_INFO("Advertising");
    }
}

static void advertising_init(void)
{
    ret_code_t             err;
    ble_advertising_init_t init;

    ble_uuid_t adv_uuids[] = {{BLE_NUS_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}};

    memset(&init, 0, sizeof(init));
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = false;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt  = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids   = adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void advertising_start(void)
{
    ret_code_t err = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err);
}

//=== Connection Parameters ===
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    ret_code_t             err;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(5000);
    cp_init.next_conn_params_update_delay  = APP_TIMER_TICKS(30000);
    cp_init.max_conn_params_update_count   = 3;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err);
}

//=== SAADC ===
static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        // Two channels: index[0] = NTC1, index[1] = NTC2
        int16_t ntc1 = p_event->data.done.p_buffer[0];
        int16_t ntc2 = p_event->data.done.p_buffer[1];

        // Log locally
        NRF_LOG_INFO("NTC1: %d, NTC2: %d", ntc1, ntc2);

        // Prepare ASCII payload
        char msg[32];
        uint16_t len = (uint16_t)snprintf(msg, sizeof(msg),
                                         "N1:%d,N2:%d\r\n", ntc1, ntc2);

        // Send over BLE NUS if connected
        if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            ret_code_t err = ble_nus_data_send(&m_nus,
                                               (uint8_t *)msg,
                                               &len,
                                               m_conn_handle);
            if (err != NRF_ERROR_INVALID_STATE &&
                err != NRF_ERROR_RESOURCES   &&
                err != NRF_ERROR_NOT_FOUND)
            {
                APP_ERROR_CHECK(err);
            }
        }

        // Re-queue buffers
        nrfx_saadc_buffer_set(saadc_buf1, SAADC_BUFFER_SIZE);
        nrfx_saadc_buffer_set(saadc_buf2, SAADC_BUFFER_SIZE);
    }
}

static void saadc_init(void)
{
    ret_code_t err;

    // Init SAADC
    err = nrfx_saadc_init(NRFX_SAADC_CONFIG_IRQ_PRIORITY);
    APP_ERROR_CHECK(err);

    // Channel 0: NTC1, Channel 1: NTC2
    nrfx_saadc_channel_config_t ch0 = NRFX_SAADC_DEFAULT_CHANNEL_SE(NTC1_AIN, 0);
    nrfx_saadc_channel_config_t ch1 = NRFX_SAADC_DEFAULT_CHANNEL_SE(NTC2_AIN, 1);

    err = nrfx_saadc_channel_init(0, &ch0);
    APP_ERROR_CHECK(err);
    err = nrfx_saadc_channel_init(1, &ch1);
    APP_ERROR_CHECK(err);

    // Simple mode, two buffers, 10-bit resolution
    err = nrfx_saadc_simple_mode_set((1UL << 0) | (1UL << 1),
                                     NRF_SAADC_RESOLUTION_10BIT,
                                     saadc_event_handler);
    APP_ERROR_CHECK(err);

    // Allocate buffers
    err = nrfx_saadc_buffer_set(saadc_buf1, SAADC_BUFFER_SIZE);
    APP_ERROR_CHECK(err);
    err = nrfx_saadc_buffer_set(saadc_buf2, SAADC_BUFFER_SIZE);
    APP_ERROR_CHECK(err);
}

//=== Idle / Power management ===
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        sd_app_evt_wait();
    }
}

//=== Main ===
int main(void)
{
    // Initialize
    log_init();
    timers_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    saadc_init();

    // Configure GPIOs
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_cfg_output(SR_RESET_PIN);
    nrf_gpio_cfg_output(NTC_EN_PIN);
    nrf_gpio_pin_clear(SR_RESET_PIN);
    nrf_gpio_pin_set(NTC_EN_PIN);

    // Start advertising
    advertising_start();

    uint32_t sample_timer = 0;
    uint32_t blink_timer  = 0;

    // Main loop
    while (true)
    {
        // Trigger SAADC every SAADC_SAMPLE_INTERVAL_MS
        if (++sample_timer >= (SAADC_SAMPLE_INTERVAL_MS / 100))
        {
            sample_timer = 0;
            nrfx_saadc_mode_trigger();
        }

        // Blink LED at 5 Hz (toggle every 100ms)
        if (++blink_timer >= 1)
        {
            blink_timer = 0;
            nrf_gpio_pin_toggle(LED_PIN);
        }

        idle_state_handle();
    }
}

//=== BLE Stack Initialization ===
static void ble_stack_init(void)
{
    ret_code_t err;

    // Enable SoftDevice
    err = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err);

    // Configure the BLE stack using the default settings
    uint32_t ram_start = 0;
    err = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err);

    // Enable BLE stack
    err = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err);
}
