#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrfx_saadc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "app_error.h"

// Pin definitions (nRF52840)
#define LED_PIN 24        // P0.24 for LED
#define SR_RESET_PIN 13   // P0.13 for reset (pin 33)
#define NTC_EN 29         // P0.29 for NTC enable (pin 8, AIN5)
#define NTC1_AIN NRF_SAADC_INPUT_AIN6  // P0.30 (pin 10)
#define NTC2_AIN NRF_SAADC_INPUT_AIN7  // P0.31 (pin 9)
#define SAADC_SAMPLE_INTERVAL_MS 100   // Sample every 100 ms (10 Hz)
#define SAADC_BUFFER_SIZE 2            // One sample per channel

// BLE definitions
#define DEVICE_NAME "NTC_Sensor"                           // Advertised device name
#define APP_BLE_CONN_CFG_TAG 1                             // Connection configuration tag
#define NTC_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN   // Custom UUID type
#define NTC_READINGS_CHAR_UUID 0x1234                      // Short UUID for NTC characteristic
#define APP_ADV_INTERVAL 300                               // Advertising interval (300 * 0.625 ms = 187.5 ms)
#define APP_ADV_TIMEOUT_IN_SECONDS 0                       // No timeout (advertise forever)
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) // Min connection interval (100 ms)
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) // Max connection interval (200 ms)
#define SLAVE_LATENCY 0                                    // Connection slave latency
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)  // Connection supervisory timeout (4 s)

// Global variables for SAADC
static nrf_saadc_value_t saadc_buffer_1[SAADC_BUFFER_SIZE];
static nrf_saadc_value_t saadc_buffer_2[SAADC_BUFFER_SIZE];
static bool ntc_sampling_enabled = true;
static uint32_t number_counter = 0;

// BLE global variables
NRF_BLE_GATT_DEF(m_gatt);                              // GATT module instance
BLE_ADVERTISING_DEF(m_advertising);                    // Advertising module instance
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; // Connection handle
static ble_uuid_t m_adv_uuids[1];                      // UUIDs for advertising
static struct
{
    uint16_t service_handle;
    ble_gatts_char_handles_t ntc_readings_char_handle;
} m_ntc_service;

// Custom 128-bit UUID for NTC service
static const ble_uuid128_t ntc_service_uuid128 =
{
    .uuid128 = {0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF,
                0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF}
};

/**@brief SAADC event handler.
 */
static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    ret_code_t err_code;

    if (p_event->type == NRFX_SAADC_EVT_DONE && ntc_sampling_enabled)
    {
        number_counter++;
        nrf_saadc_value_t ntc1_value = p_event->data.done.p_buffer[0];
        nrf_saadc_value_t ntc2_value = p_event->data.done.p_buffer[1];
        NRF_LOG_INFO("Reading #%lu: NTC1 (P0.30): %d, NTC2 (P0.31): %d",
                     number_counter, ntc1_value, ntc2_value);

        // Prepare data for BLE notification (NTC1 and NTC2 as int16_t)
        uint8_t data[4];
        data[0] = (uint8_t)(ntc1_value & 0xFF);
        data[1] = (uint8_t)(ntc1_value >> 8);
        data[2] = (uint8_t)(ntc2_value & 0xFF);
        data[3] = (uint8_t)(ntc2_value >> 8);

        // Send BLE notification if connected
        if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            hvx_params.handle = m_ntc_service.ntc_readings_char_handle.value_handle;
            hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len = &(uint16_t){sizeof(data)};
            hvx_params.p_data = data;

            err_code = sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_INFO("NTC readings notified");
            }
            else if (err_code != NRF_ERROR_BUSY)
            {
                NRF_LOG_ERROR("Notification failed: %d", err_code);
            }
        }

        // Set up the next buffer (alternate between buffers)
        err_code = nrfx_saadc_buffer_set(p_event->data.done.p_buffer == saadc_buffer_1 ? saadc_buffer_2 : saadc_buffer_1, SAADC_BUFFER_SIZE);
        APP_ERROR_CHECK(err_code);
    }
    else if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
    {
        NRF_LOG_INFO("SAADC calibration complete");
    }
}

/**@brief Initialize GPIO pins.
 */
static void gpio_init(void)
{
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_cfg_output(SR_RESET_PIN);
    nrf_gpio_cfg_output(NTC_EN);
    nrf_gpio_pin_clear(SR_RESET_PIN);
    nrf_gpio_pin_set(NTC_EN);
}

/**@brief Initialize SAADC.
 */
static void saadc_init(void)
{
    ret_code_t err_code;

    // Initialize logging
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("NTC SAADC with LED, Reset, and BLE");

    // Initialize SAADC
    err_code = nrfx_saadc_init(NRFX_SAADC_CONFIG_IRQ_PRIORITY);
    APP_ERROR_CHECK(err_code);

    // Configure two channels
    nrfx_saadc_channel_t channels[2] = {
        NRFX_SAADC_DEFAULT_CHANNEL_SE(NTC1_AIN, 0),  // AIN6 (P0.30)
        NRFX_SAADC_DEFAULT_CHANNEL_SE(NTC2_AIN, 1)   // AIN7 (P0.31)
    };
    err_code = nrfx_saadc_channels_config(channels, 2);
    APP_ERROR_CHECK(err_code);

    // Configure simple mode
    err_code = nrfx_saadc_simple_mode_set((1 << 0) | (1 << 1),
                                          NRF_SAADC_RESOLUTION_10BIT,
                                          NULL,
                                          saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    // Set up two buffers for double buffering
    err_code = nrfx_saadc_buffer_set(saadc_buffer_1, SAADC_BUFFER_SIZE);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_saadc_buffer_set(saadc_buffer_2, SAADC_BUFFER_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Handle BLE stack events.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("BLE connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("BLE disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            // Restart advertising
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
            {
                NRF_LOG_INFO("Advertising timeout, restarting");
                err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Initialize BLE stack.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    // Initialize SoftDevice
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register BLE event handler
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Initialize GATT.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialize NTC service and characteristic.
 */
static void ntc_service_init(void)
{
    ret_code_t err_code;
    ble_uuid_t service_uuid;

    // Add custom base UUID
    err_code = sd_ble_uuid_vs_add(&ntc_service_uuid128, &NTC_SERVICE_UUID_TYPE);
    APP_ERROR_CHECK(err_code);

    // Set service UUID
    service_uuid.type = NTC_SERVICE_UUID_TYPE;
    service_uuid.uuid = 0x0001; // Short UUID for the service

    // Add service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &m_ntc_service.service_handle);
    APP_ERROR_CHECK(err_code);

    // Define characteristic
    ble_gatts_char_md_t char_md = {0};
    ble_gatts_attr_t attr_char_value = {0};
    ble_uuid_t char_uuid = {.uuid = NTC_READINGS_CHAR_UUID, .type = NTC_SERVICE_UUID_TYPE};
    ble_gatts_attr_md_t attr_md = {0};

    // Characteristic metadata
    char_md.char_props.read = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    // Attribute metadata
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    // Attribute value
    uint8_t initial_value[4] = {0}; // Initial NTC readings (4 bytes for two int16_t)
    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(initial_value);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(initial_value);
    attr_char_value.p_value = initial_value;

    // Add characteristic
    err_code = sd_ble_gatts_characteristic_add(m_ntc_service.service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &m_ntc_service.ntc_readings_char_handle);
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialize advertising.
 */
static void advertising_init(void)
{
    ret_code_t err_code;
    ble_advdata_t advdata = {0};
    ble_advdata_t srdata = {0};

    // Service UUID
    m_adv_uuids[0].type = NTC_SERVICE_UUID_TYPE;
    m_adv_uuids[0].uuid = 0x0001;

    // Build advertising data
    advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = 1;
    advdata.uuids_complete.p_uuids = m_adv_uuids;

    // Initialize advertising
    ble_advertising_init_t init = {0};
    init.advdata = advdata;
    init.srdata = srdata;
    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Handle connection parameters events.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Handle connection parameters errors.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Initialize connection parameters.
 */
static void conn_params_init(void)
{
    ret_code_t err_code;
    ble_conn_params_init_t cp_init = {0};

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = MSEC_TO_UNITS(5000, UNIT_10_MS);
    cp_init.next_conn_params_update_delay = MSEC_TO_UNITS(30000, UNIT_10_MS);
    cp_init.max_conn_params_update_count = 3;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialize timers.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    // Initialize timers
    timers_init();

    // Configure GPIO pins
    gpio_init();

    // Initialize BLE stack
    ble_stack_init();
    gatt_init();
    ntc_service_init();
    advertising_init();
    conn_params_init();

    // Initialize SAADC
    saadc_init();

    // Start advertising
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    // Initialize counters
    uint32_t reset_counter = 0;
    uint32_t led_counter = 0;
    uint32_t saadc_counter = 0;

    const uint32_t reset_max_count = 100000; // 100 * 100 ms = 10 seconds
    const uint32_t led_interval = 2;         // 2 * 100 ms = 200 ms
    const uint32_t saadc_interval = 1;       // 1 * 100 ms = 100 ms

    while (1)
 {
        // Increment counters
        reset_counter++;
        led_counter++;
        saadc_counter++;

        // Toggle LED every 200 ms
        if (led_counter >= led_interval)
        {
            nrf_gpio_pin_toggle(LED_PIN);
            led_counter = 0;
        }

        // Sample ADC every 100 ms if enabled
        if (ntc_sampling_enabled && saadc_counter >= saadc_interval)
        {
            ret_code_t err_code = nrfx_saadc_mode_trigger();
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("SAADC trigger failed: %d", err_code);
            }
            saadc_counter = 0;
        }

        // After 10 seconds, update pins and stop sampling
        if (reset_counter >= reset_max_count)
        {
            nrf_gpio_pin_set(SR_RESET_PIN); // Set reset pin high
            nrf_gpio_pin_clear(NTC_EN);     // Set NTC_EN low
            ntc_sampling_enabled = false;   // Stop ADC sampling
            reset_counter = reset_max_count; // Stop counter
        }

        // Process RTT logs
        while (NRF_LOG_PROCESS() != NRF_SUCCESS);

        // Wait for 100 ms
        nrf_delay_ms(100);
    }
}