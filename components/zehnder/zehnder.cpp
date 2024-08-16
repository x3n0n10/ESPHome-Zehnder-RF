#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

static const char *const TAG = "zehnder";

// Define RF payload structures
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
  uint8_t rx_type;
  uint8_t rx_id;
  uint8_t tx_type;
  uint8_t tx_id;
  uint8_t ttl;
  uint8_t command;
  uint8_t parameter_count;

  union {
    uint8_t parameters[9];
    RfPayloadFanSetSpeed setSpeed;
    RfPayloadFanSetTimer setTimer;
    RfPayloadNetworkJoinRequest networkJoinRequest;
    RfPayloadNetworkJoinOpen networkJoinOpen;
    RfPayloadFanSettings fanSettings;
    RfPayloadNetworkJoinAck networkJoinAck;
  } payload;
} RfFrame;

// Constructor
ZehnderRF::ZehnderRF() : rf_(nullptr), interval_(1000), speed_count_(0), state_(StateIdle), newSpeed(0), newTimer(0), newSetting(false) {}

// Initialize the RF and other settings
void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ZehnderRF...");

  if (this->rf_) {
    nrf905::Config rfConfig;
    rfConfig.band = true;
    rfConfig.channel = 118;
    rfConfig.crc_enable = true;
    rfConfig.crc_bits = 16;
    rfConfig.tx_power = 10;
    rfConfig.rx_power = nrf905::PowerNormal;
    rfConfig.rx_address = 0x89816EA9;
    rfConfig.rx_address_width = 4;
    rfConfig.rx_payload_width = 16;
    rfConfig.tx_address_width = 4;
    rfConfig.tx_payload_width = 16;
    rfConfig.xtal_frequency = 16000000;
    rfConfig.clkOutFrequency = nrf905::ClkOut500000;
    rfConfig.clkOutEnable = false;

    this->rf_->updateConfig(&rfConfig);
    this->rf_->writeTxAddress(0x89816EA9);

    this->rf_->setOnTxReady([this]() {
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
}

// Dump configuration to logs
void ZehnderRF::dump_config() {
  ESP_LOGCONFIG(TAG, "ZehnderRF:");
  ESP_LOGCONFIG(TAG, "  Update Interval: %d ms", this->interval_);
  if (this->rf_) {
    // Since get_mode and get_data_rate are not available, just show basic info
    ESP_LOGCONFIG(TAG, "  RF configuration is available");
  } else {
    ESP_LOGCONFIG(TAG, "  RF configuration not available");
  }
}

// Get fan traits
fan::FanTraits ZehnderRF::get_traits() {
  fan::FanTraits traits;
  traits.set_speed_count(this->speed_count_);
  traits.supports_speed(true);
  return traits;
}

// Handle control commands
void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    this->state_ = *call.get_state() == fan::FanState::ON ? StateActive : StateIdle;
  }

  if (call.get_speed().has_value()) {
    this->setSpeed(*call.get_speed(), 0); // Assuming no timer
  }
}

// Set speed of the fan
void ZehnderRF::setSpeed(const uint8_t speed, const uint8_t timer) {
  this->newSpeed = speed;
  this->newTimer = timer;
  this->newSetting = true;
}

// Main loop
void ZehnderRF::loop() {
  if (this->rfState_ == RfStateTxBusy && (millis() - msgSendTime_ > FAN_REPLY_TIMEOUT)) {
    this->rfState_ = RfStateIdle;
    this->update_error_status();
  }

  if (this->newSetting) {
    this->sendSpeed();
    this->newSetting = false;
  }
}

// Send fan speed settings
void ZehnderRF::sendSpeed() {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;

  pFrame->command = FAN_SET_SPEED_TIMER;
  pFrame->parameter_count = sizeof(RfPayloadFanSetTimer);
  pFrame->payload.setTimer.speed = this->newSpeed;
  pFrame->payload.setTimer.timer = this->newTimer;

  this->sendRfFrame(FAN_UNIT_TYPE_REMOTE, FAN_NETWORK_LINK_TTL);
}

// Update error status
void ZehnderRF::update_error_status() {
  // Placeholder for error conditions
  if (/* Fan malfunction condition */) {
    error_code = E03_FAN_MALFUNCTION;
  } else if (filter_runtime > /* threshold */) {
    error_code = E05_FILTER_REPLACEMENT_NEEDED;
  } else {
    error_code = NO_ERROR;
  }
}

// Handle received RF data
void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *) pData;

  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest:
      if (pResponse->command == FAN_NETWORK_JOIN_OPEN) {
        ESP_LOGD(TAG, "Discovery: Found unit with ID 0x%02X on network 0x%08X",
                 pResponse->tx_id, pResponse->payload.networkJoinOpen.networkId);

        this->config_.fan_main_unit_type = pResponse->tx_type;
        this->config_.fan_main_unit_id = pResponse->tx_id;
        this->config_.fan_networkId = pResponse->payload.networkJoinOpen.networkId;

        this->pref_.save(&this->config_);

        RfFrame txFrame;
        txFrame.command = FAN_NETWORK_JOIN_REQUEST;
        txFrame.parameter_count = sizeof(RfPayloadNetworkJoinRequest);
        txFrame.payload.networkJoinRequest.networkId = pResponse->payload.networkJoinOpen.networkId;

        this->sendRfFrame(FAN_UNIT_TYPE_REMOTE, FAN_NETWORK_LINK_TTL);
        this->state_ = StateDiscoveryWaitForLinkAck;

        nrf905::Config rfConfig = this->rf_->getConfig();
        rfConfig.rx_address = this->config_.fan_networkId;
        this->rf_->updateConfig(&rfConfig);
        this->rf_->writeTxAddress(this->config_.fan_networkId);
      }
      break;

    case StateDiscoveryWaitForLinkAck:
      if (pResponse->command == FAN_NETWORK_JOIN_ACK) {
        ESP_LOGD(TAG, "Discovery: Ack received from unit 0x%02X", pResponse->tx_id);
        this->state_ = StateIdle;
      }
      break;

    case StateIdle:
      break;

    default:
      ESP_LOGE(TAG, "Unhandled state %d", this->state_);
      break;
  }
}

// Send RF frame
void ZehnderRF::sendRfFrame(const uint8_t txType, const uint8_t txId) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;

  pFrame->tx_type = txType;
  pFrame->tx_id = txId;
  pFrame->ttl = 3; // Set appropriate TTL
  pFrame->command = FAN_COMMAND;

  this->rf_->send(reinterpret_cast<uint8_t *>(pFrame), sizeof(*pFrame));
}

} // namespace zehnder
} // namespace esphome
