# H-Link Driver Robustness Enhancements

This document summarizes the improvements made to the H-Link driver to match production quality standards from the ESPHome reference implementation.

## Implemented Enhancements

### 1. Response Timeout Tracking
**Purpose:** Detect communication failures and timeout conditions reliably.

**Changes:**
- Added timing tracking variables:
  - `s_last_frame_sent_ms`: Tracks when last frame was sent
  - `s_timeout_counter_started_at_ms`: Marks start of timeout period
  - `s_timeout_counter_active`: Flag indicating active timeout monitoring
  
- Added helper functions:
  - `millis()`: FreeRTOS tick-based millisecond timer
  - `reached_timeout_threshold()`: Checks if communication timeout (1000ms) exceeded

**Benefits:**
- Proper timeout detection across request/response cycles
- Prevents indefinite waiting on unresponsive AC units
- Enables recovery from communication failures

### 2. Partial Response Detection
**Purpose:** Detect and handle incomplete UART frames properly.

**Changes:**
- Modified `hlink_read_response()` to return `hlink_frame_status_t` enum instead of `esp_err_t`
- New return statuses:
  - `HLINK_FRAME_NOTHING`: No data received (complete timeout)
  - `HLINK_FRAME_PARTIAL`: Incomplete frame received
  - `HLINK_FRAME_OK`: Valid OK response with complete frame
  - `HLINK_FRAME_NG`: Valid NG (error) response from AC unit
  - `HLINK_FRAME_INVALID`: Malformed or corrupted frame

**Benefits:**
- Distinguishes between different failure modes
- Enables appropriate error handling and recovery strategies
- Better debugging through detailed frame status logging

### 3. Checksum Validation
**Purpose:** Verify data integrity of all responses from AC unit.

**Changes:**
- Enhanced `parse_mt_response()` to:
  - Extract address from response
  - Extract data payload
  - Extract received checksum from `C=YYYY` field
  - Calculate expected checksum **correctly for responses** (only checksums P_VALUE data, not address)
  - Compare and validate checksums
  - Return `HLINK_FRAME_INVALID` on mismatch

**Important Note:**
The H-Link protocol uses **different checksum calculations** for requests vs responses:
- **Request checksum:** `0xFFFF - address_high - address_low - data_bytes`
- **Response checksum:** `0xFFFF - data_bytes` (address is NOT included)

This is a critical detail discovered from the ESPHome reference implementation.

**Benefits:**
- Detects corrupted data from electrical noise or transmission errors
- Prevents acting on invalid state information
- Increases reliability in noisy electrical environments

### 4. Frame Interval Timing
**Purpose:** Enforce minimum delay between requests to respect AC unit timing requirements.

**Changes:**
- Added constant: `MIN_INTERVAL_BETWEEN_REQUESTS = 60ms`
- Added `can_send_next_frame()` function that checks elapsed time since last transmission
- Modified both `hlink_send_mt_request()` and `hlink_send_st_request()` to:
  - Wait until minimum interval has passed before sending
  - Update `s_last_frame_sent_ms` timestamp after transmission

**Benefits:**
- Prevents overwhelming AC unit with rapid requests
- Ensures AC unit has time to process commands
- Matches timing requirements discovered in reverse engineering

### 5. Model Name Reading
**Purpose:** Read and store AC unit model identification.

**Changes:**
- Added `hlink_read_model_name()` function that:
  - Reads from feature address `0x0900`
  - Parses ASCII model name string
  - Stores in `hlink_state_t.model_name[32]`
  - Logs model name for debugging

- Added public API in `hlink_driver.h`

**Benefits:**
- Enables model-specific behavior if needed
- Useful for debugging and support
- Allows displaying model in user interfaces

## Updated Function Signatures

### Modified Functions
```c
// Old: static esp_err_t hlink_read_response(char *buf, size_t buf_size, uint32_t timeout_ms);
// New:
static hlink_frame_status_t hlink_read_response(char *buf, size_t buf_size, uint32_t timeout_ms);

// Old: static esp_err_t parse_mt_response(const char *response, uint8_t *data, size_t *data_len);
// New:
static hlink_frame_status_t parse_mt_response(const char *response, uint8_t *data, size_t *data_len);
```

### New Public API
```c
esp_err_t hlink_read_model_name(void);
```

### New Helper Functions
```c
static uint32_t millis(void);
static bool reached_timeout_threshold(void);
static bool can_send_next_frame(void);
```

## Integration Updates

All setter and getter functions have been updated to handle the new `hlink_frame_status_t` return type:
- `hlink_set_power()`
- `hlink_set_mode()`
- `hlink_set_temperature()`
- `hlink_set_fan_mode()`
- `hlink_set_swing_mode()`
- `hlink_set_remote_lock()`
- `hlink_set_beeper()`
- `hlink_set_leave_home()`
- `hlink_reset_filter_warning()`
- `hlink_request_status_update()`

## Testing Recommendations

1. **Timeout Testing**: Disconnect H-Link bus and verify timeout detection works correctly
2. **Checksum Testing**: Inject noise on UART lines to verify checksum validation catches errors
3. **Timing Testing**: Send rapid commands and verify 60ms minimum interval is enforced
4. **Model Name**: Call `hlink_read_model_name()` during initialization and verify correct model string

## References

- ESPHome H-Link AC Component: https://github.com/lumixen/esphome-hlink-ac
- Hackaday H-Link Reverse Engineering: https://hackaday.io/project/179773-hitachi-room-air-conditioner-to-modbus-rtualyator
- H-Link Protocol Documentation: MT/ST command format with checksum validation

## Compatibility

These changes maintain backward compatibility with the existing Zigbee integration (`esp_zb_hlink.c`). No changes required in other parts of the codebase.
