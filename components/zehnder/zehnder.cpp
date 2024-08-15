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

ZehnderRF::ZehnderRF() {}

fan::FanTraits ZehnderRF::get_traits() {
  return fan::FanTraits(false, true, false, this->speed_count_);
}

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    bool fan_state = *call.get_state();
    // Map boolean to one of the existing State enum values
    this->state_ = fan_state ? StateIdle : StateStartup; // Adjust based on your use case
    ESP_LOGD(TAG, "Control has state: %u", this->state_);
  }
  if (call.get_speed().has_value()) {
    this->speed = *call.get_speed();
    ESP_LOGD(TAG, "Control has speed: %u", this->speed);
  }

  switch (this->state_) {
    case StateIdle:
      this->setSpeed(this->speed, 0);
      this->lastFanQuery_ = millis();
      break;

    default:
      break;
  }

  this->publish_state();
}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

  memset(&this->config_, 0, sizeof(Config));

  uint32_t hash = fnv1_hash("zehnderrf");
  this->pref_ = global_preferences->make_preference<Config>(hash, true);
  if (this->pref_.load(&this->config_)) {
    ESP_LOGD(TAG, "Config load ok");
  }

  nrf905::Config rfConfig;
  rfConfig = this->rf_->getConfig();

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

  this->speed_count_ = 4;

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

void ZehnderRF::dump_config() {
  ESP_LOGCONFIG(TAG, "Zehnder Fan config:");
  ESP_LOGCONFIG(TAG, "  Polling interval   %u", this->interval_);
  ESP_LOGCONFIG(TAG, "  Fan networkId      0x%08X", this->config_.fan_networkId);
  ESP_LOGCONFIG(TAG, "  Fan my device type 0x%02X", this->config_.fan_my_device_type);
  ESP_LOGCONFIG(TAG, "  Fan my device id   0x%02X", this->config_.fan_my_device_id);
  ESP_LOGCONFIG(TAG, "  Fan main_unit type 0x%02X", this->config_.fan_main_unit_type);
  ESP_LOGCONFIG(TAG, "  Fan main unit id   0x%02X", this->config_.fan_main_unit_id);
  ESP_LOGCONFIG(TAG, "  Speed count        %u", this->speed_count_);
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = reinterpret_cast<const RfFrame *>(pData);

  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest:
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_REQUEST:
          ESP_LOGD(TAG, "Discovery: Request received from unit 0x%02X", pResponse->tx_id);
          this->state_ = StateDiscoveryWaitForLinkAck;
          break;
      }
      break;

    case StateDiscoveryWaitForLinkAck:
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_ACK:
          ESP_LOGD(TAG, "Discovery: Ack received from unit 0x%02X", pResponse->tx_id);
          this->state_ = StateIdle;
          break;
      }
      break;

    case StateIdle:
      switch (pResponse->command) {
        case FAN_UPDATE_SETTINGS:
          if ((pResponse->tx_type == this->config_.fan_main_unit_type) && (pResponse->tx_id == this->config_.fan_main_unit_id)) {
            this->fanSettingsReceived(pResponse->payload.fanSettings.speed, pResponse->payload.fanSettings.voltage,
                                      pResponse->payload.fanSettings.timer);
          }
          break;
      }
      break;
  }
}

void ZehnderRF::rfHandler() {
  switch (this->rfState_) {
    case RfStateIdle:
      break;

    case RfStateRxBusy:
      if ((millis() - this->msgSendTime_) > MAX_TRANSMIT_TIME) {
        this->rfState_ = RfStateIdle;
      }
      break;

    case RfStateRxWait:
      if ((millis() - this->msgSendTime_) > MAX_TRANSMIT_TIME) {
        if (this->retries_-- > 0) {
          this->rfState_ = RfStateTxBusy;
        } else {
          this->rfState_ = RfStateIdle;
        }
      }
      break;

    case RfStateTxBusy:
      if ((millis() - this->msgSendTime_) > MAX_TRANSMIT_TIME) {
        this->rfState_ = RfStateIdle;
      }
      break;
  }
}

void ZehnderRF::sendRfFrame(const uint8_t deviceType, const uint8_t ttl) {
  RfFrame *const pFrame = reinterpret_cast<RfFrame *>(this->_txFrame);

  pFrame->rx_type = this->config_.fan_main_unit_type;
  pFrame->rx_id = this->config_.fan_main_unit_id;
  pFrame->tx_type = deviceType;
  pFrame->tx_id = this->config_.fan_my_device_id;
  pFrame->ttl = ttl;

  this->append_crc_to_payload(reinterpret_cast<uint8_t *>(pFrame), sizeof(RfFrame));

  this->rf_->writeTxPayload(reinterpret_cast<const uint8_t *>(pFrame), sizeof(RfFrame));

  this->retries_ = 3;  // Send 3 times to be sure
  this->rfState_ = RfStateTxBusy;
}

void ZehnderRF::setSpeed(const uint8_t speed, const uint8_t timer) {
  RfFrame *const pFrame = reinterpret_cast<RfFrame *>(this->_txFrame);

  ESP_LOGD(TAG, "Set fan speed 0x%02X timer %u", speed, timer);

  pFrame->command = FAN_SET_SPEED_TIMER;
  pFrame->parameter_count = sizeof(RfPayloadFanSetTimer);
  pFrame->payload.setTimer.speed = speed;
  pFrame->payload.setTimer.timer = timer;

  this->sendRfFrame(FAN_UNIT_TYPE_REMOTE, FAN_TTL);

  this->state_ = StateWaitSetSpeedConfirm;
}

void ZehnderRF::fanSettingsReceived(const uint8_t speed, const uint8_t voltage, const uint8_t timer) {
  ESP_LOGD(TAG, "Fan settings speed %u, voltage %u, timer %u", speed, voltage, timer);
}

void ZehnderRF::queryDevice() {
  RfFrame *const pFrame = reinterpret_cast<RfFrame *>(this->_txFrame);

  pFrame->command = FAN_UPDATE_SETTINGS;
  pFrame->parameter_count = 0;

  this->sendRfFrame(FAN_UNIT_TYPE_REMOTE, FAN_TTL);

  this->lastFanQuery_ = millis();  // Update time
  this->state_ = StateWaitQueryForUpdate;
}

uint8_t ZehnderRF::createDeviceID() {
  uint32_t v = 0;
  uint32_t v1, v2;

  std::string name = this->get_name();
  v1 = fnv1_hash(name);

  // Placeholder for MAC address or other unique identifier
  std::string mac_address = "00:00:00:00:00:00";
  v2 = fnv1_hash(mac_address);

  v = fnv1_hash(name);

  return ((v1 + v2) % 0xFF);
}

uint16_t ZehnderRF::calculate_crc16(uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void ZehnderRF::append_crc_to_payload(uint8_t *payload, size_t length) {
  uint16_t crc = this->calculate_crc16(payload, length - 2);
  payload[length - 2] = crc & 0xFF;
  payload[length - 1] = (crc >> 8) & 0xFF;
}

void ZehnderRF::discoveryStart(uint8_t deviceId) {
  ESP_LOGD(TAG, "Starting discovery with parameter: %u", deviceId);

  // Initialize the discovery frame
  RfFrame *const pFrame = reinterpret_cast<RfFrame *>(this->_txFrame);
  
  pFrame->rx_type = FAN_TYPE_BROADCAST;  // Broadcast to all devices
  pFrame->rx_id = 0xFF;                  // Broadcast to all IDs
  pFrame->tx_type = FAN_TYPE_MAIN_UNIT;  // Sending from main unit
  pFrame->tx_id = this->config_.fan_my_device_id; // Set to device's ID
  pFrame->ttl = FAN_TTL;                 // Set time-to-live for the frame
  pFrame->command = FAN_NETWORK_JOIN_OPEN;
  pFrame->parameter_count = sizeof(RfPayloadNetworkJoinOpen);
  pFrame->payload.networkJoinOpen.networkId = this->config_.fan_networkId;

  // Append CRC to the payload
  this->append_crc_to_payload(reinterpret_cast<uint8_t *>(pFrame), sizeof(RfFrame));

  // Send the frame
  this->rf_->writeTxPayload(reinterpret_cast<const uint8_t *>(pFrame), sizeof(RfFrame));

  // Update state
  this->rfState_ = RfStateTxBusy;
  this->msgSendTime_ = millis();  // Record the time when the message is sent
}

}  // namespace zehnder
}  // namespace esphome
