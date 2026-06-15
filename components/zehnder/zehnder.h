#ifndef __COMPONENT_ZEHNDER_H__
#define __COMPONENT_ZEHNDER_H__

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/nrf905/nRF905.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace zehnder {

// Bump this whenever the component code changes so you can confirm from Home
// Assistant / the logs which build is actually running on the device.
#define ZEHNDER_RF_VERSION "0.1.0"

#define FAN_FRAMESIZE 16        // Each frame consists of 16 bytes
#define FAN_TX_FRAMES 4         // Retransmit every transmitted frame 4 times
#define FAN_TX_RETRIES 10       // Retry transmission 10 times if no reply is received
#define FAN_TTL 250             // 0xFA, default time-to-live for a frame
#define FAN_REPLY_TIMEOUT 2000  // Wait 2000ms for receiving a reply
#define FAN_RETRY_DELAY 150     // Non-blocking pause between transmit retries

// Safety timeouts so the state machine can never permanently wedge (which would
// otherwise require a manual reboot of the ESP).
#define FAN_TX_TIMEOUT 2000               // Max time to wait for a TxReady before resetting the radio
#define FAN_STATE_WATCHDOG_TIMEOUT 60000  // Force recovery if stuck out of idle this long

// nRF905 RF settings used to talk to the Zehnder/BUVA fan network.
#define FAN_RF_CHANNEL 118          // nRF905 channel
#define FAN_RF_TX_POWER 10          // nRF905 TX power in dBm
#define FAN_DEFAULT_RF_ADDRESS 0x89816EA9  // Address used before a network is paired

// Timing of the main loop's periodic work.
#define FAN_STARTUP_DELAY_MS 15000          // Wait after boot before talking to the fan
#define FAN_AIRWAY_TIMEOUT_MS 5000          // Give up if the airway never goes free
#define FAN_FILTER_QUERY_INTERVAL_MS 600000  // Poll filter status every 10 minutes
#define FAN_ERROR_QUERY_INTERVAL_MS 300000   // Poll error status every 5 minutes

/* Fan device types */
enum {
  FAN_TYPE_BROADCAST = 0x00,       // Broadcast to all devices
  FAN_TYPE_MAIN_UNIT = 0x01,       // Fans
  FAN_TYPE_REMOTE_CONTROL = 0x03,  // Remote controls
  FAN_TYPE_CO2_SENSOR = 0x18
};  // CO2 sensors

/* Fan commands */
enum {
  FAN_FRAME_SETVOLTAGE = 0x01,  // Set speed (voltage / percentage)
  FAN_FRAME_SETSPEED = 0x02,    // Set speed (preset)
  FAN_FRAME_SETTIMER = 0x03,    // Set speed with timer
  FAN_NETWORK_JOIN_REQUEST = 0x04,
  FAN_FRAME_SETSPEED_REPLY = 0x05,
  FAN_NETWORK_JOIN_OPEN = 0x06,
  FAN_TYPE_FAN_SETTINGS = 0x07,  // Current settings, sent by fan in reply to 0x01, 0x02, 0x10
  FAN_FRAME_0B = 0x0B,
  FAN_NETWORK_JOIN_ACK = 0x0C,
  // FAN_NETWORK_JOIN_FINISH = 0x0D,
  FAN_TYPE_QUERY_NETWORK = 0x0D,
  FAN_TYPE_QUERY_DEVICE = 0x10,
  FAN_FRAME_SETVOLTAGE_REPLY = 0x1D,

  // New diagnostic commands (actual command codes would need to be discovered)
  FAN_TYPE_QUERY_ERROR_STATUS = 0x30,    // Request error codes
  FAN_TYPE_ERROR_STATUS_RESPONSE = 0x31, // Response with error codes
  FAN_TYPE_QUERY_FILTER_STATUS = 0x32,   // Request filter status
  FAN_TYPE_FILTER_STATUS_RESPONSE = 0x33 // Response with filter status
};

/* Fan speed presets */
enum {
  FAN_SPEED_AUTO = 0x00,    // Off:      0% or  0.0 volt
  FAN_SPEED_LOW = 0x01,     // Low:     30% or  3.0 volt
  FAN_SPEED_MEDIUM = 0x02,  // Medium:  50% or  5.0 volt
  FAN_SPEED_HIGH = 0x03,    // High:    90% or  9.0 volt
  FAN_SPEED_MAX = 0x04
};  // Max:    100% or 10.0 volt

#define NETWORK_LINK_ID 0xA55A5AA5
#define NETWORK_DEFAULT_ID 0xE7E7E7E7
#define FAN_JOIN_DEFAULT_TIMEOUT 10000

typedef enum { ResultOk, ResultBusy, ResultFailure } Result;

/* ------------------------------------------------------------------------- *
 *  Over-the-air protocol frame structures
 *
 *  Every frame is FAN_FRAMESIZE (16) bytes: a 7-byte header followed by a
 *  9-byte payload whose meaning depends on the command field.
 * ------------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinOpen;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinRequest;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinAck;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t voltage;
  uint8_t timer;
} RfPayloadFanSettings;

typedef struct __attribute__((packed)) {
  uint8_t speed;
} RfPayloadFanSetSpeed;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t timer;
} RfPayloadFanSetTimer;

typedef struct __attribute__((packed)) {
  uint8_t errorCount;     // Number of active errors
  uint8_t errorCodes[5];  // Array of error codes
  uint8_t errorSeverity;  // Severity level (warning/critical)
} RfPayloadErrorStatus;

typedef struct __attribute__((packed)) {
  uint16_t totalRunHours;          // Total operation hours
  uint16_t filterRunHours;         // Hours since last filter change
  uint8_t filterPercentRemaining;  // Filter life remaining percentage
} RfPayloadFilterStatus;

typedef struct __attribute__((packed)) {
  uint8_t rx_type;          // 0x00 RX Type
  uint8_t rx_id;            // 0x01 RX ID
  uint8_t tx_type;          // 0x02 TX Type
  uint8_t tx_id;            // 0x03 TX ID
  uint8_t ttl;              // 0x04 Time-To-Live
  uint8_t command;          // 0x05 Frame type
  uint8_t parameter_count;  // 0x06 Number of parameters

  union {
    uint8_t parameters[9];                          // 0x07 - 0x0F Depends on command
    RfPayloadFanSetSpeed setSpeed;                  // Command 0x02
    RfPayloadFanSetTimer setTimer;                  // Command 0x03
    RfPayloadNetworkJoinRequest networkJoinRequest; // Command 0x04
    RfPayloadNetworkJoinOpen networkJoinOpen;       // Command 0x06
    RfPayloadFanSettings fanSettings;               // Command 0x07
    RfPayloadNetworkJoinAck networkJoinAck;         // Command 0x0C
  } payload;
} RfFrame;

class ZehnderRF : public Component, public fan::Fan {
 public:
  ZehnderRF();

  void setup() override;

  // Setup things
  void set_rf(nrf905::nRF905 *const pRf) { rf_ = pRf; }

  void set_update_interval(const uint32_t interval) { interval_ = interval; }

  // Sensors
  void set_filter_remaining_sensor(sensor::Sensor *sensor) { filter_remaining_sensor_ = sensor; }
  void set_filter_runtime_sensor(sensor::Sensor *sensor) { filter_runtime_sensor_ = sensor; }
  void set_error_count_sensor(sensor::Sensor *sensor) { error_count_sensor_ = sensor; }
  void set_error_code_sensor(text_sensor::TextSensor *sensor) { error_code_sensor_ = sensor; }

  void dump_config() override;
  void set_config(const uint32_t fan_networkId,
                  const uint8_t  fan_my_device_type,
                  const uint8_t  fan_my_device_id,
                  const uint8_t  fan_main_unit_type,
                  const uint8_t  fan_main_unit_id);

  fan::FanTraits get_traits() override;
  int get_speed_count() { return this->speed_count_; }

  // Returns the component version string (see ZEHNDER_RF_VERSION). Handy for a
  // template text_sensor so the running version is visible in Home Assistant.
  const char *get_version() const { return ZEHNDER_RF_VERSION; }

  void loop() override;

  void control(const fan::FanCall &call) override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  void setSpeed(const uint8_t speed, const uint8_t timer = 0);

  // NOTE: `timer` and `voltage` are read directly from the YAML lambdas, so do
  // not rename them. `timer` is a flag meaning "a countdown is currently active";
  // `voltage` is the fan's reported output percentage.
  bool timer;
  int voltage;

 protected:
  void queryDevice(void);
  void queryErrorStatus(void);
  void queryFilterStatus(void);

  uint8_t createDeviceID(void);
  void discoveryStart(const uint8_t deviceId);

  Result startTransmit(const uint8_t *const pData, const int8_t rxRetries = -1,
                       const std::function<void(void)> callback = NULL);
  void rfComplete(void);
  void rfHandler(void);
  void rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength);

  // Returns true if a received frame is addressed to this device.
  bool addressedToUs(const RfFrame *const pResponse) const;
  // Applies a fan-settings frame to our published state (speed/timer/voltage).
  void handleFanSettings(const RfFrame *const pResponse);

  // Per-state receive handlers, dispatched from rfHandleReceived() based on the
  // current protocol state. Keeping one handler per state keeps each readable.
  void rfHandleLinkRequest(const RfFrame *const pResponse);
  void rfHandleJoinResponse(const RfFrame *const pResponse);
  void rfHandleJoinComplete(const RfFrame *const pResponse);
  void rfHandleFanSettingsResponse(const RfFrame *const pResponse);
  void rfHandleFilterStatusResponse(const RfFrame *const pResponse);
  void rfHandleErrorStatusResponse(const RfFrame *const pResponse);
  void rfHandleSetSpeedResponse(const RfFrame *const pResponse);

  typedef enum {
    StateStartup,
    StateStartDiscovery,
    StateDiscoveryWaitForLinkRequest,
    StateDiscoveryWaitForJoinResponse,
    StateDiscoveryJoinComplete,

    StateIdle,
    StateWaitQueryResponse,
    StateWaitSetSpeedResponse,
    StateWaitSetSpeedConfirm,

    // New states
    StateWaitFilterStatusResponse,
    StateWaitErrorStatusResponse,

    StateNrOf  // Keep last
  } State;
  // Protocol state machine. NOTE: distinct from the inherited fan::Fan member
  // `state` (the on/off flag) -- this drives the RF conversation with the fan.
  State fsmState_{StateStartup};
  int speed_count_{};

  nrf905::nRF905 *rf_;
  uint32_t interval_;

  // Sensors
  sensor::Sensor *filter_remaining_sensor_{nullptr};
  sensor::Sensor *filter_runtime_sensor_{nullptr};
  sensor::Sensor *error_count_sensor_{nullptr};
  text_sensor::TextSensor *error_code_sensor_{nullptr};

  uint8_t _txFrame[FAN_FRAMESIZE];

  ESPPreferenceObject pref_;

  typedef struct {
    uint32_t fan_networkId;      // Fan (Zehnder/BUVA) network ID
    uint8_t fan_my_device_type;  // Fan (Zehnder/BUVA) device type
    uint8_t fan_my_device_id;    // Fan (Zehnder/BUVA) device ID
    uint8_t fan_main_unit_type;  // Fan (Zehnder/BUVA) main unit type
    uint8_t fan_main_unit_id;    // Fan (Zehnder/BUVA) main unit ID
  } Config;
  Config config_;

  uint32_t lastFanQuery_{0};
  uint32_t lastFilterQuery_{0};
  uint32_t lastErrorQuery_{0};
  std::function<void(void)> onReceiveTimeout_ = NULL;

  uint32_t msgSendTime_{0};
  uint32_t airwayFreeWaitTime_{0};
  uint32_t txStartTime_{0};        // Time the radio entered TxBusy (for TX timeout)
  uint32_t lastStateIdleTime_{0};  // Last time the main state machine was idle (for watchdog)
  uint32_t retryWaitTime_{0};      // Start of the non-blocking pause between retries
  int8_t retries_{-1};

  uint8_t newSpeed{0};
  uint8_t newTimer{0};
  bool newSetting{false};

  typedef enum {
    RfStateIdle,            // Idle state
    RfStateWaitAirwayFree,  // wait for airway free
    RfStateTxBusy,          //
    RfStateRxWait,
    RfStateRetryWait,       // Non-blocking pause before retrying a transmit
  } RfState;
  RfState rfState_{RfStateIdle};
};

}  // namespace zehnder
}  // namespace esphome

#endif /* __COMPONENT_ZEHNDER_H__ */
