#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000

static const char *const TAG = "zehnder";

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
  uint8_t rx_type;          // 0x00 RX Type
  uint8_t rx_id;            // 0x01 RX ID
  uint8_t tx_type;          // 0x02 TX Type
  uint8_t tx_id;            // 0x03 TX ID
  uint8_t ttl;              // 0x04 Time-To-Live
  uint8_t command;          // 0x05 Frame type
  uint8_t parameter_count;  // 0x06 Number of parameters

  union {
    uint8_t parameters[9];                           // 0x07 - 0x0F Depends on command
    RfPayloadFanSetSpeed setSpeed;                   // Command 0x02
    RfPayloadFanSetTimer setTimer;                   // Command 0x03
    RfPayloadNetworkJoinRequest networkJoinRequest;  // Command 0x04
    RfPayloadNetworkJoinOpen networkJoinOpen;        // Command 0x06
    RfPayloadFanSettings fanSettings;                // Command 0x07
    RfPayloadNetworkJoinAck networkJoinAck;          // Command 0x0C
  } payload;
} RfFrame;

ZehnderRF::ZehnderRF(void) {}

fan::FanTraits ZehnderRF::get_traits() { return fan::FanTraits(false, true, false, this->speed_count_); }

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    this->state = *call.get_state();
    ESP_LOGD(TAG, "Control has state: %u", this->state);
  }
  if (call.get_speed().has_value()) {
    this->speed = *call.get_speed();
    ESP_LOGD(TAG, "Control has speed: %u", this->speed);
  }

  switch (this->state_) {
    case StateIdle:
      // Set speed
      this->setSpeed(this->state ? this->speed : 0x00, 0);

      this->lastFanQuery_ = millis();  // Update time
      break;

    default:
      break;
  }

  this->publish_state();
}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

  // Clear config
  memset(&this->config_, 0, sizeof(Config));

  uint32_t hash = fnv1_hash("zehnderrf");
  this->pref_ = global_preferences->make_preference<Config>(hash, true);
  if (this->pref_.load(&this->config_)) {
    ESP_LOGD(TAG, "Config load ok");
  }

  // Set nRF905 config
  nrf905::Config rfConfig;
  rfConfig = this->rf_->getConfig();

  rfConfig.band = true;
  rfConfig.channel = 118;

  // // CRC 16
  rfConfig.crc_enable = true;
  rfConfig.crc_bits = 16;

  // // TX power 10
  rfConfig.tx_power = 10;

  // // RX power normal
  rfConfig.rx_power = nrf905::PowerNormal;

  rfConfig.rx_address = 0x89816EA9;  // ZEHNDER_NETWORK_LINK_ID;
  rfConfig.rx_address_width = 4;
  rfConfig.rx_payload_width = 16;

  rfConfig.tx_address_width = 4;
  rfConfig.tx_payload_width = 16;

  rfConfig.xtal_frequency = 16000000;  // defaults for now
  rfConfig.clkOutFrequency = nrf905::ClkOut500000;
  rfConfig.clkOutEnable = false;

  // Write config back
  this->rf_->updateConfig(&rfConfig);
  this->rf_->writeTxAddress(0x89816EA9);

  this->speed_count_ = 4;

  this->lastFilterQuery_ = 0;
  this->lastErrorQuery_ = 0;

  this->rf_->setOnTxReady([this](void) {
    ESP_LOGD(TAG, "Tx Ready");
    if (this->rfState_ == RfStateTxBusy) {
      if (this->retries_ >= 0) {
        this->msgSendTime_ = millis();
        this->rfState_ = RfStateRxWait;
      } else {
        this->rfState_ = RfStateIdle;
      }
    }
  });

  this->rf_->setOnRxComplete([this](const uint8_t *const pData, const uint8_t dataLength) {
    ESP_LOGV(TAG, "Received frame");
    this->rfHandleReceived(pData, dataLength);
  });
}

void ZehnderRF::dump_config(void) {
  ESP_LOGCONFIG(TAG, "Zehnder Fan config:");
  ESP_LOGCONFIG(TAG, "  Polling interval   %u", this->interval_);
  ESP_LOGCONFIG(TAG, "  Fan networkId      0x%08X", this->config_.fan_networkId);
  ESP_LOGCONFIG(TAG, "  Fan my device type 0x%02X", this->config_.fan_my_device_type);
  ESP_LOGCONFIG(TAG, "  Fan my device id   0x%02X", this->config_.fan_my_device_id);
  ESP_LOGCONFIG(TAG, "  Fan main_unit type 0x%02X", this->config_.fan_main_unit_type);
  ESP_LOGCONFIG(TAG, "  Fan main unit id   0x%02X", this->config_.fan_main_unit_id);

  // Log diagnostic sensors
  if (this->filter_remaining_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Filter remaining sensor: YES");
  }
  if (this->filter_runtime_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Filter runtime sensor: YES");
  }
  if (this->error_count_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Error count sensor: YES");
  }
  if (this->error_code_sensor_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Error code sensor: YES");
  }
}

void ZehnderRF::loop(void) {
  uint8_t deviceId;
  nrf905::Config rfConfig;

  // Run RF handler
  this->rfHandler();

  switch (this->state_) {
    case StateStartup:
      // Wait until started up
      if (millis() > 15000) {
        // Discovery?
        if ((this->config_.fan_networkId == 0x00000000) || (this->config_.fan_my_device_type == 0) ||
            (this->config_.fan_my_device_id == 0) || (this->config_.fan_main_unit_type == 0) ||
            (this->config_.fan_main_unit_id == 0)) {
          ESP_LOGD(TAG, "Invalid config, start paring");

          this->state_ = StateStartDiscovery;
        } else {
          ESP_LOGD(TAG, "Config data valid, start polling");

          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = this->config_.fan_networkId;
          this->rf_->updateConfig(&rfConfig);
          this->rf_->writeTxAddress(this->config_.fan_networkId);

          // Start with query
          this->queryDevice();
        }
      }
      break;

    case StateStartDiscovery:
      deviceId = this->createDeviceID();
      this->discoveryStart(deviceId);

      // For now just set TX
      break;

    case StateWaitSetSpeedConfirm:
      if (this->rfState_ == RfStateIdle) {
        // When done, return to idle
        this->state_ = StateIdle;
      }

      case StateIdle:
        if (newSetting == true) {
          this->setSpeed(newSpeed, newTimer);
        } else {
          // Existing fan query
          if ((millis() - this->lastFanQuery_) > this->interval_) {
            this->queryDevice();
          }

          // Add filter query every 10 minutes if sensors are connected
          if (((millis() - this->lastFilterQuery_) > 600000) &&
              (this->filter_remaining_sensor_ != nullptr || this->filter_runtime_sensor_ != nullptr)) {
            this->queryFilterStatus();
          }

          // Add error query every 5 minutes if sensors are connected
          if (((millis() - this->lastErrorQuery_) > 300000) &&
              (this->error_count_sensor_ != nullptr || this->error_code_sensor_ != nullptr)) {
            this->queryErrorStatus();
          }
        }
        break;


    default:
      break;
  }
}

void ZehnderRF::queryErrorStatus(void) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper

  // Try different command codes
  // Current command: 0x30
  // Let's try: 0x31, 0x11, 0x20, or 0x40
  // Comment/uncomment the line below to change the command code
  uint8_t test_command = 0x31;  // Change this value to test different codes

  ESP_LOGI(TAG, "Query error status (command: 0x%02X)", test_command);

  this->lastErrorQuery_ = millis();  // Update time

  // Clear frame data
  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);

  // Build frame
  pFrame->rx_type = this->config_.fan_main_unit_type;
  pFrame->rx_id = this->config_.fan_main_unit_id;
  pFrame->tx_type = this->config_.fan_my_device_type;
  pFrame->tx_id = this->config_.fan_my_device_id;
  pFrame->ttl = FAN_TTL;
  pFrame->command = test_command;  // Use the test command
  pFrame->parameter_count = 0x00;  // No parameters

  ESP_LOGD(TAG, "Error query frame: type:0x%02X id:0x%02X -> type:0x%02X id:0x%02X cmd:0x%02X",
           pFrame->tx_type, pFrame->tx_id, pFrame->rx_type, pFrame->rx_id, pFrame->command);

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Error status query timeout");

    // Notify UI about failure via sensors if available
    if (this->error_code_sensor_ != nullptr) {
      this->error_code_sensor_->publish_state("Query Timeout");
    }

    this->state_ = StateIdle;
  });

  this->state_ = StateWaitErrorStatusResponse;
}

void ZehnderRF::queryFilterStatus(void) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper

  ESP_LOGI(TAG, "Attempting to query filter status");
  ESP_LOGI(TAG, "  Sender Type: 0x%02X", this->config_.fan_my_device_type);
  ESP_LOGI(TAG, "  Sender ID: 0x%02X", this->config_.fan_my_device_id);
  ESP_LOGI(TAG, "  Receiver Type: 0x%02X", this->config_.fan_main_unit_type);
  ESP_LOGI(TAG, "  Receiver ID: 0x%02X", this->config_.fan_main_unit_id);

  this->lastFilterQuery_ = millis();  // Update time

  // Clear frame data
  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);

  // Build frame
  pFrame->rx_type = this->config_.fan_main_unit_type;
  pFrame->rx_id = this->config_.fan_main_unit_id;
  pFrame->tx_type = this->config_.fan_my_device_type;
  pFrame->tx_id = this->config_.fan_my_device_id;
  pFrame->ttl = FAN_TTL;
  pFrame->command = FAN_TYPE_QUERY_FILTER_STATUS;
  pFrame->parameter_count = 0x00;  // No parameters

  ESP_LOGI(TAG, "  Sending Filter Query Command: 0x%02X", pFrame->command);

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Filter status query TIMEOUT");
    ESP_LOGW(TAG, "  No response received within retry limit");
    this->state_ = StateIdle;
  });

  this->state_ = StateWaitFilterStatusResponse;
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *) pData;
  RfFrame *const pTxFrame = (RfFrame *) this->_txFrame;  // frame helper
  nrf905::Config rfConfig;

  // Comprehensive Frame Logging
  ESP_LOGI(TAG, "===== RECEIVED RF FRAME =====");
  ESP_LOGI(TAG, "Current State: 0x%02X", this->state_);
  ESP_LOGI(TAG, "Frame Details:");
  ESP_LOGI(TAG, "  Total Data Length: %u bytes", dataLength);
  ESP_LOGI(TAG, "  Receiver Type: 0x%02X", pResponse->rx_type);
  ESP_LOGI(TAG, "  Receiver ID: 0x%02X", pResponse->rx_id);
  ESP_LOGI(TAG, "  Transmitter Type: 0x%02X", pResponse->tx_type);
  ESP_LOGI(TAG, "  Transmitter ID: 0x%02X", pResponse->tx_id);
  ESP_LOGI(TAG, "  Command Code: 0x%02X", pResponse->command);
  ESP_LOGI(TAG, "  Parameter Count: %u", pResponse->parameter_count);

  // Hex Dump of Full Payload
  ESP_LOGI(TAG, "Full Frame Hex Dump:");
  char hexDump[256] = {0};
  size_t hexOffset = 0;
  for (uint8_t i = 0; i < dataLength && hexOffset < sizeof(hexDump) - 5; i++) {
    hexOffset += snprintf(hexDump + hexOffset, sizeof(hexDump) - hexOffset,
                          "%02X ", pData[i]);
  }
  ESP_LOGI(TAG, "  %s", hexDump);

  // Payload Byte Analysis
  ESP_LOGI(TAG, "Payload Byte Analysis:");
  for (uint8_t i = 0; i < pResponse->parameter_count && i < 9; i++) {
    ESP_LOGI(TAG, "  Byte %u: 0x%02X (Dec: %u)",
             i,
             pResponse->payload.parameters[i],
             pResponse->payload.parameters[i]);
  }

  // Known Command Code Interpretation
  const char* commandDescription = "Unknown";
  switch (pResponse->command) {
    case FAN_FRAME_SETSPEED:
      commandDescription = "Set Speed (0x02)";
      break;
    case FAN_FRAME_SETTIMER:
      commandDescription = "Set Timer (0x03)";
      break;
    case FAN_NETWORK_JOIN_REQUEST:
      commandDescription = "Network Join Request (0x04)";
      break;
    case FAN_FRAME_SETSPEED_REPLY:
      commandDescription = "Set Speed Reply (0x05)";
      break;
    case FAN_NETWORK_JOIN_OPEN:
      commandDescription = "Network Join Open (0x06)";
      break;
    case FAN_TYPE_FAN_SETTINGS:
      commandDescription = "Fan Settings (0x07)";
      break;
    case FAN_FRAME_0B:
      commandDescription = "Frame 0B - Link Acknowledge (0x0B)";
      break;
    case FAN_NETWORK_JOIN_ACK:
      commandDescription = "Network Join Ack (0x0C)";
      break;
    case FAN_TYPE_QUERY_NETWORK:
      commandDescription = "Query Network (0x0D)";
      break;
    case FAN_TYPE_QUERY_DEVICE:
      commandDescription = "Query Device (0x10)";
      break;
    case FAN_FRAME_SETVOLTAGE_REPLY:
      commandDescription = "Set Voltage Reply (0x1D)";
      break;
    case FAN_TYPE_QUERY_ERROR_STATUS:
      commandDescription = "Query Error Status (0x30)";
      break;
    case FAN_TYPE_ERROR_STATUS_RESPONSE:
      commandDescription = "Error Status Response (0x31)";
      break;
    case FAN_TYPE_QUERY_FILTER_STATUS:
      commandDescription = "Query Filter Status (0x32)";
      break;
    case FAN_TYPE_FILTER_STATUS_RESPONSE:
      commandDescription = "Filter Status Response (0x33)";
      break;
  }
  ESP_LOGI(TAG, "Command Interpretation: %s", commandDescription);

  // Continue with the original state machine processing
  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest:
      ESP_LOGD(TAG, "DiscoverStateWaitForLinkRequest");
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_OPEN:  // Received linking request from main unit
          ESP_LOGD(TAG, "Discovery: Found unit type 0x%02X (%s) with ID 0x%02X on network 0x%08X", pResponse->tx_type,
                   pResponse->tx_type == FAN_TYPE_MAIN_UNIT ? "Main" : "?", pResponse->tx_id,
                   pResponse->payload.networkJoinOpen.networkId);

          this->rfComplete();

          (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

          // Found a main unit, so send a join request
          pTxFrame->rx_type = FAN_TYPE_MAIN_UNIT;  // Set type to main unit
          pTxFrame->rx_id = pResponse->tx_id;      // Set ID to the ID of the main unit
          pTxFrame->tx_type = this->config_.fan_my_device_type;
          pTxFrame->tx_id = this->config_.fan_my_device_id;
          pTxFrame->ttl = FAN_TTL;
          pTxFrame->command = FAN_NETWORK_JOIN_REQUEST;  // Request to connect to network
          pTxFrame->parameter_count = sizeof(RfPayloadNetworkJoinOpen);
          // Request to connect to the received network ID
          pTxFrame->payload.networkJoinRequest.networkId = pResponse->payload.networkJoinOpen.networkId;

          // Store for later
          this->config_.fan_networkId = pResponse->payload.networkJoinOpen.networkId;
          this->config_.fan_main_unit_type = pResponse->tx_type;
          this->config_.fan_main_unit_id = pResponse->tx_id;

          // Update address
          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = pResponse->payload.networkJoinOpen.networkId;
          this->rf_->updateConfig(&rfConfig, NULL);
          this->rf_->writeTxAddress(pResponse->payload.networkJoinOpen.networkId, NULL);

          // Send response frame
          this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
            ESP_LOGW(TAG, "Query Timeout");
            this->state_ = StateStartDiscovery;
          });

          this->state_ = StateDiscoveryWaitForJoinResponse;
          break;

        default:
          ESP_LOGD(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X", pResponse->command,
                   pResponse->tx_id);
          break;
      }
      break;

    case StateWaitFilterStatusResponse:
      if ((pResponse->rx_type == this->config_.fan_my_device_type) &&  // If type
          (pResponse->rx_id == this->config_.fan_my_device_id)) {      // and id match, it is for us
        switch (pResponse->command) {
          case FAN_TYPE_FILTER_STATUS_RESPONSE: {
            const RfPayloadFilterStatus* filterStatus =
                reinterpret_cast<const RfPayloadFilterStatus*>(&pResponse->payload);

            ESP_LOGD(TAG, "Received filter status; total hours: %u, filter hours: %u, remaining: %u%%",
                     filterStatus->totalRunHours, filterStatus->filterRunHours,
                     filterStatus->filterPercentRemaining);

            this->rfComplete();

            // Update sensor values if you've added them
            if (this->filter_remaining_sensor_ != nullptr) {
              this->filter_remaining_sensor_->publish_state(filterStatus->filterPercentRemaining);
            }
            if (this->filter_runtime_sensor_ != nullptr) {
              this->filter_runtime_sensor_->publish_state(filterStatus->filterRunHours);
            }

            this->state_ = StateIdle;
            break;
          }

          default:
            ESP_LOGD(TAG, "Received unexpected frame; type 0x%02X from ID 0x%02X",
                     pResponse->command, pResponse->tx_id);
            break;
        }
      } else {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X",
                 pResponse->command, pResponse->tx_id, pResponse->tx_type);
      }
      break;

    default:
      ESP_LOGD(TAG, "Received frame from unknown device in unknown state; type 0x%02X from ID 0x%02X type 0x%02X",
               pResponse->command, pResponse->tx_id, pResponse->tx_type);
      break;
}
}

static uint8_t minmax(const uint8_t value, const uint8_t min, const uint8_t max) {
  if (value <= min) {
    return min;
  } else if (value >= max) {
    return max;
  } else {
    return value;
  }
}

uint8_t ZehnderRF::createDeviceID(void) {
  uint8_t random = (uint8_t) random_uint32();
  // Generate random device_id; don't use 0x00 and 0xFF

  // TODO: there's a 1 in 255 chance that the generated ID matches the ID of the main unit. Decide how to deal
  // withthis (some sort of ping discovery?)

  return minmax(random, 1, 0xFE);
}

void ZehnderRF::queryDevice(void) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper

  ESP_LOGD(TAG, "Query device");

  this->lastFanQuery_ = millis();  // Update time

  // Clear frame data
  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);

  // Build frame
  pFrame->rx_type = this->config_.fan_main_unit_type;
  pFrame->rx_id = this->config_.fan_main_unit_id;
  pFrame->tx_type = this->config_.fan_my_device_type;
  pFrame->tx_id = this->config_.fan_my_device_id;
  pFrame->ttl = FAN_TTL;
  pFrame->command = FAN_TYPE_QUERY_DEVICE;
  pFrame->parameter_count = 0x00;  // No parameters

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Query Timeout");
    this->state_ = StateIdle;
  });

  this->state_ = StateWaitQueryResponse;
}

void ZehnderRF::setSpeed(const uint8_t paramSpeed, const uint8_t paramTimer) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper
  uint8_t speed = paramSpeed;
  uint8_t timer = paramTimer;

  if (speed > this->speed_count_) {
    ESP_LOGW(TAG, "Requested speed too high (%u)", speed);
    speed = this->speed_count_;
  }

  ESP_LOGD(TAG, "Set speed: 0x%02X; Timer %u minutes", speed, timer);

  if (this->state_ == StateIdle) {
    (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

    // Build frame
    pFrame->rx_type = this->config_.fan_main_unit_type;
    pFrame->rx_id = 0x00;  // Broadcast
    pFrame->tx_type = this->config_.fan_my_device_type;
    pFrame->tx_id = this->config_.fan_my_device_id;
    pFrame->ttl = FAN_TTL;

    if (timer == 0) {
      pFrame->command = FAN_FRAME_SETSPEED;
      pFrame->parameter_count = sizeof(RfPayloadFanSetSpeed);
      pFrame->payload.setSpeed.speed = speed;
    } else {
      pFrame->command = FAN_FRAME_SETTIMER;
      pFrame->parameter_count = sizeof(RfPayloadFanSetTimer);
      pFrame->payload.setTimer.speed = speed;
      pFrame->payload.setTimer.timer = timer;
    }

    ESP_LOGD(TAG, "Waiting for initial RF stabilization...");
    delay(500);  // Wait 500ms for RF to stabilize

    this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
      ESP_LOGW(TAG, "Set speed timeout");
      this->state_ = StateIdle;
    });

    newSetting = false;
    this->state_ = StateWaitSetSpeedResponse;
  } else {
    ESP_LOGD(TAG, "Invalid state, I'm trying later again");
    newSpeed = speed;
    newTimer = timer;
    newSetting = true;
  }
}

void ZehnderRF::discoveryStart(const uint8_t deviceId) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper
  nrf905::Config rfConfig;

  ESP_LOGD(TAG, "Start discovery with ID %u", deviceId);

  this->config_.fan_my_device_type = FAN_TYPE_REMOTE_CONTROL;
  this->config_.fan_my_device_id = deviceId;

  // Build frame
  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

  // Set payload, available for linking
  pFrame->rx_type = 0x04;
  pFrame->rx_id = 0x00;
  pFrame->tx_type = this->config_.fan_my_device_type;
  pFrame->tx_id = this->config_.fan_my_device_id;
  pFrame->ttl = FAN_TTL;
  pFrame->command = FAN_NETWORK_JOIN_ACK;
  pFrame->parameter_count = sizeof(RfPayloadNetworkJoinAck);
  pFrame->payload.networkJoinAck.networkId = NETWORK_LINK_ID;

  // Set RX and TX address
  rfConfig = this->rf_->getConfig();
  rfConfig.rx_address = NETWORK_LINK_ID;
  this->rf_->updateConfig(&rfConfig, NULL);
  this->rf_->writeTxAddress(NETWORK_LINK_ID, NULL);

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Start discovery timeout");
    this->state_ = StateStartDiscovery;
  });

  // Update state
  this->state_ = StateDiscoveryWaitForLinkRequest;
}

Result ZehnderRF::startTransmit(const uint8_t *const pData, const int8_t rxRetries,
                                const std::function<void(void)> callback) {
  Result result = ResultOk;
  unsigned long startTime;
  bool busy = true;

  if (this->rfState_ != RfStateIdle) {
    ESP_LOGW(TAG, "TX still ongoing");
    result = ResultBusy;
  } else {
    this->onReceiveTimeout_ = callback;
    this->retries_ = rxRetries;

    // Write data to RF
    // if (pData != NULL) {  // If frame given, load it in the nRF. Else use previous TX payload
    // ESP_LOGD(TAG, "Write payload");
    this->rf_->writeTxPayload(pData, FAN_FRAMESIZE);  // Use framesize
    // }

    this->rfState_ = RfStateWaitAirwayFree;
    this->airwayFreeWaitTime_ = millis();
  }

  return result;
}

void ZehnderRF::rfComplete(void) {
  this->retries_ = -1;  // Disable this->retries_
  this->rfState_ = RfStateIdle;
}

void ZehnderRF::rfHandler(void) {
  switch (this->rfState_) {
    case RfStateIdle:
      break;

    case RfStateWaitAirwayFree:
      if ((millis() - this->airwayFreeWaitTime_) > 5000) {
        ESP_LOGW(TAG, "Airway too busy, giving up");
        this->rfState_ = RfStateIdle;

        if (this->onReceiveTimeout_ != NULL) {
          this->onReceiveTimeout_();
        }
      } else if (this->rf_->airwayBusy() == false) {
        ESP_LOGD(TAG, "Start TX");
        this->rf_->startTx(FAN_TX_FRAMES, nrf905::Receive);  // After transmit, wait for response

        this->rfState_ = RfStateTxBusy;
      }
      break;

    case RfStateTxBusy:
      break;

    case RfStateRxWait:
      if ((this->retries_ >= 0) && ((millis() - this->msgSendTime_) > FAN_REPLY_TIMEOUT)) {
        ESP_LOGD(TAG, "Receive timeout");

        if (this->retries_ > 0) {
          --this->retries_;
          ESP_LOGD(TAG, "No data received, retry again (left: %u)", this->retries_);

          // Add a short delay between retries
          delay(150);

          this->rfState_ = RfStateWaitAirwayFree;
          this->airwayFreeWaitTime_ = millis();
        } else if (this->retries_ == 0) {
          // Oh oh, ran out of options

          ESP_LOGD(TAG, "No messages received, giving up now...");
          if (this->onReceiveTimeout_ != NULL) {
            this->onReceiveTimeout_();
          }

          // Add a longer delay before moving to idle
          delay(250);

          // Back to idle
          this->rfState_ = RfStateIdle;
        }
      }
      break;

    default:
      break;
  }
}

}  // namespace zehnder
}  // namespace esphome
