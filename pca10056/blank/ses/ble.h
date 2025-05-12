#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrfx_saadc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define LED_PIN 24        // P0.24 for LED
#define SR_RESET_PIN 13   // P0.13 for reset (pin 33)
#define NTC_EN 29         // P0.29 for NTC enable (pin 8, AIN5)
#define NTC1_AIN NRF_SAADC_INPUT_AIN6  // P0.30 (pin 10)
#define NTC2_AIN NRF_SAADC_INPUT_AIN7  // P0.31 (pin 9)
#define SAADC_SAMPLE_INTERVAL_MS 1000   // Sample every 100 ms (10 Hz)
#define SAADC_BUFFER_SIZE 2            // One sample per channel
#define READINGS_LIMIT 15              // Number of readings to collect
#define MATCH_THRESHOLD_MIN 7          // Minimum matching readings
#define MATCH_THRESHOLD_MAX 8          // Maximum matching readings

// Global variables for SAADC
static nrf_saadc_value_t saadc_buffer_1[SAADC_BUFFER_SIZE];
static nrf_saadc_value_t saadc_buffer_2[SAADC_BUFFER_SIZE];
static bool ntc_sampling_enabled = true;
static uint32_t number_counter = 0; 
static nrf_saadc_value_t ntc1_readings[READINGS_LIMIT]; // Store NTC1 readings
static nrf_saadc_value_t ntc2_readings[READINGS_LIMIT]; // Store NTC2 readings
static uint32_t reading_index = 0; // Current index in readings array

// Function to check if readings match within threshold
static bool check_readings_match(nrf_saadc_value_t *readings, uint32_t count)
{
    uint32_t match_count = 1; // Count of identical readings
    nrf_saadc_value_t reference = readings[0]; // Use first reading as reference

    // Count how many readings match the first one
    for (uint32_t i = 1; i < count; i++)
    {
        if (readings[i] == reference)
        {
            match_count++;
        }
    }

    // Check if match count is within threshold (7 to 8)
    return (match_count >= MATCH_THRESHOLD_MIN && match_count <= MATCH_THRESHOLD_MAX);
}

// SAADC event handler
static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    ret_code_t err_code;

    if (p_event->type == NRFX_SAADC_EVT_DONE && ntc_sampling_enabled)
    {
        number_counter++;  // Increment the reading counter

        // Store readings
        ntc1_readings[reading_index] = p_event->data.done.p_buffer[0];
        ntc2_readings[reading_index] = p_event->data.done.p_buffer[1];
        reading_index++;

        // Log ADC readings
        NRF_LOG_INFO("Reading #%lu: NTC1 (P0.30): %d, NTC2 (P0.31): %d",
                     number_counter,
                     p_event->data.done.p_buffer[0],
                     p_event->data.done.p_buffer[1]);

        // Check if we've collected 15 readings
        if (reading_index >= READINGS_LIMIT)
        {
            // Check if either NTC1 or NTC2 readings match the criteria
            bool ntc1_match = check_readings_match(ntc1_readings, READINGS_LIMIT);
            bool ntc2_match = check_readings_match(ntc2_readings, READINGS_LIMIT);

            if (ntc1_match || ntc2_match)
            {
                // Set SR_RESET_PIN to low
                nrf_gpio_pin_clear(SR_RESET_PIN);
                NRF_LOG_INFO("7-8 identical readings detected in 15 samples. SR_RESET_PIN set to LOW");
                // Optionally stop sampling
                ntc_sampling_enabled = false;
            }
            else
            {
                // Clear buffers and start new collection
                memset(ntc1_readings, 0, sizeof(ntc1_readings));
                memset(ntc2_readings, 0, sizeof(ntc2_readings));
                reading_index = 0;
                NRF_LOG_INFO("No 7-8 identical readings in 15 samples. Buffers cleared, restarting collection");
            }
        }
    }
    else if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
    {
        NRF_LOG_INFO("SAADC calibration complete");
    }
}

// Initialize SAADC
static void saadc_init(void)
{
    ret_code_t err_code;

    // Initialize logging
    err_code = NRF_LOG_INIT(NULL);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Log init failed: %d", err_code);
        return;
    }
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("NTC SAADC with LED and Reset/Button Control");

    // Initialize SAADC
    err_code = nrfx_saadc_init(NRFX_SAADC_CONFIG_IRQ_PRIORITY);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("SAADC init failed: %d", err_code);
        return;
    }

    // Configure two channels
    nrfx_saadc_channel_t channels[2] = {
        NRFX_SAADC_DEFAULT_CHANNEL_SE(NTC1_AIN, 0),  // AIN6 (P0.30)
        NRFX_SAADC_DEFAULT_CHANNEL_SE(NTC2_AIN, 1)   // AIN7 (P0.31)
    };
    err_code = nrfx_saadc_channels_config(channels, 2);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Channel config failed: %d", err_code);
        return;
    }

    // Configure simple mode
    err_code = nrfx_saadc_simple_mode_set((1 << 0) | (1 << 1),
                                         NRF_SAADC_RESOLUTION_10BIT,
                                         NULL,
                                         saadc_event_handler);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Simple mode set failed: %d", err_code);
        return;
    }

    // Set up two buffers for double buffering
    err_code = nrfx_saadc_buffer_set(saadc_buffer_1, SAADC_BUFFER_SIZE);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Buffer 1 set failed: %d", err_code);
        return;
    }
    err_code = nrfx_saadc_buffer_set(saadc_buffer_2, SAADC_BUFFER_SIZE);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Buffer 2 set failed: %d", err_code);
        return;
    }
}

int main(void)
{
    // Configure GPIO pins
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_cfg_output(SR_RESET_PIN); // Initially configure as output
    nrf_gpio_cfg_output(NTC_EN);
    nrf_gpio_pin_clear(SR_RESET_PIN); // Set reset pin low
    nrf_gpio_pin_set(NTC_EN);         // Set NTC_EN high

    // Initialize SAADC
    saadc_init();

    // Initialize counters and button state
    uint32_t reset_counter = 0;      // For reset pin and NTC_EN timing
    uint32_t led_counter = 0;        // For LED blinking
    uint32_t saadc_counter = 0;      // For SAADC sampling
    static bool last_button_state = true; // Track button state (true = not pressed, false = pressed)
    static bool sr_reset_state = false;  // Track SR_RESET_PIN output state (false = low, true = high)

    const uint32_t reset_max_count = 180000; // 10 seconds (100 * 100 ms)
    const uint32_t led_interval = 2;      // 2 * 100 ms = 200 ms
    const uint32_t saadc_interval = 10;   // 10 * 100 ms = 1000 ms

    while (1)
    {
        // Increment counters
        reset_counter++;
        led_counter++;
        saadc_counter++;

        // Check button on SR_RESET_PIN
        // Temporarily configure SR_RESET_PIN as input with pull-up to read button
        nrf_gpio_cfg_input(SR_RESET_PIN, NRF_GPIO_PIN_PULLUP);
        bool current_button_state = nrf_gpio_pin_read(SR_RESET_PIN); // 0 = pressed, 1 = not pressed
        // Restore SR_RESET_PIN as output and set to last known state
        nrf_gpio_cfg_output(SR_RESET_PIN);
        if (sr_reset_state)
            nrf_gpio_pin_set(SR_RESET_PIN);
        else
            nrf_gpio_pin_clear(SR_RESET_PIN);

        // Detect button press (transition from not pressed to pressed)
        if (current_button_state == false && last_button_state == true)
        {
            // Debounce: Wait to ensure stable press (10 µF capacitor may slow transition)
            nrf_delay_ms(100); // Increased debounce time due to 10 µF capacitor
            // Re-check button state after debounce
            nrf_gpio_cfg_input(SR_RESET_PIN, NRF_GPIO_PIN_PULLUP);
            if (nrf_gpio_pin_read(SR_RESET_PIN) == false)
            {
                // Toggle SR_RESET_PIN state (set high on first press, low on next, etc.)
                sr_reset_state = !sr_reset_state;
                // Set the new state
                nrf_gpio_cfg_output(SR_RESET_PIN);
                if (sr_reset_state)
                {
                    nrf_gpio_pin_set(SR_RESET_PIN);
                    NRF_LOG_INFO("Button pressed: SR_RESET_PIN set to HIGH (1)");
                }
                else
                {
                    nrf_gpio_pin_clear(SR_RESET_PIN);
                    NRF_LOG_INFO("Button pressed: SR_RESET_PIN set to LOW (0)");
                }
            }
            else
            {
                // Restore output state if debounce fails
                nrf_gpio_cfg_output(SR_RESET_PIN);
                if (sr_reset_state)
                    nrf_gpio_pin_set(SR_RESET_PIN);
                else
                    nrf_gpio_pin_clear(SR_RESET_PIN);
            }
        }
        last_button_state = current_button_state;

        // Toggle LED every 200 ms
        if (led_counter >= led_interval)
        {
            nrf_gpio_pin_toggle(LED_PIN);
            led_counter = 0;
        }

        // Sample ADC every 1000 ms if enabled
        if (ntc_sampling_enabled && saadc_counter >= saadc_interval)
        {
            ret_code_t err_code = nrfx_saadc_mode_trigger();
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("SAADC trigger failed: %d", err_code);
            }
            saadc_counter = 0;
        }

        // After 10 seconds, update pins and stop sampling (original logic)
        if (reset_counter == reset_max_count)
        {
            nrf_gpio_pin_set(SR_RESET_PIN); // Set reset pin high
            sr_reset_state = true;          // Update state tracking
            nrf_gpio_pin_clear(NTC_EN);     // Set NTC_EN low
            ntc_sampling_enabled = false;   // Stop ADC sampling
            reset_counter = reset_max_count; // Stop counter
            NRF_LOG_INFO("10 seconds reached: SR_RESET_PIN set to HIGH, ADC sampling stopped");
        }

        // Process RTT logs
        while (NRF_LOG_PROCESS() != NRF_SUCCESS);

        // Wait for 100 ms
        nrf_delay_ms(100);
    }
}