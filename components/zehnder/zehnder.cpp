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

    case StateIdle:
      if (newSetting == true) {
        this->setSpeed(newSpeed, newTimer);
      } else {
        if ((millis() - this->lastFanQuery_) > this->interval_) {
          this->queryDevice();
        }
      }
      break;

    case StateWaitSetSpeedConfirm:
      if (this->rfState_ == RfStateIdle) {
        // When done, return to idle
        this->state_ = StateIdle;
      }

    default:
      break;
  }
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *) pData;
  RfFrame *const pTxFrame = (RfFrame *) this->_txFrame;  // frame helper
  nrf905::Config rfConfig;

  ESP_LOGD(TAG, "Current state: 0x%02X", this->state_);
  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest:
      ESP_LOGD(TAG, "DiscoverStateWaitForLinkRequest");
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_OPEN:  // Received linking request from main unit
          ESP_LOGD(TAG, "Discovery: Found unit type 0x%02X (%s) with ID 0x%02X on network 0x%08X", pResponse->tx_type,
                   pResponse->tx_type == FAN_UNIT_TYPE_MAIN ? "Main" : "Unknown", pResponse->tx_id,
                   pResponse->payload.networkJoinOpen.networkId);

          // Save to config
          this->config_.fan_main_unit_type = pResponse->tx_type;
          this->config_.fan_main_unit_id = pResponse->tx_id;
          this->config_.fan_networkId = pResponse->payload.networkJoinOpen.networkId;
          this->config_.fan_my_device_id = pTxFrame->rx_id;

          // Save config
          this->pref_.save(&this->config_);

          // ACK
          pTxFrame->command = FAN_NETWORK_JOIN_REQUEST;
          pTxFrame->parameter_count = sizeof(RfPayloadNetworkJoinRequest);
          pTxFrame->payload.networkJoinRequest.networkId = pResponse->payload.networkJoinOpen.networkId;

          this->sendRfFrame(FAN_UNIT_TYPE_REMOTE, FAN_NETWORK_LINK_TTL);

          this->state_ = StateDiscoveryWaitForLinkAck;

          // Update config
          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = this->config_.fan_networkId;
          this->rf_->updateConfig(&rfConfig);
          this->rf_->writeTxAddress(this->config_.fan_networkId);

          break;
      }
      break;

    case StateDiscoveryWaitForLinkAck:
      ESP_LOGD(TAG, "DiscoverStateWaitForLinkAck");
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_ACK:  // Received confirmation from main unit
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

void ZehnderRF::rfHandler(void) {
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
          this->rf_->send();
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
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;

  pFrame->rx_type = this->config_.fan_main_unit_type;
  pFrame->rx_id = this->config_.fan_main_unit_id;
  pFrame->tx_type = deviceType;
  pFrame->tx_id = this->config_.fan_my_device_id;
  pFrame->ttl = ttl;

  this->rf_->writeTxPayload(pFrame, sizeof(RfFrame));
  this->rf_->send();

  this->retries_ = 3;  // Send 3 times to be sure
  this->rfState_ = RfStateTxBusy;
}

void ZehnderRF::setSpeed(const uint8_t speed, const uint8_t timer) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;

  ESP_LOGD(TAG, "Set fan speed 0x%02X timer %u", speed, timer);

  pFrame->command = FAN_SET_SPEED_TIMER;
  pFrame->parameter_count = sizeof(RfPayloadFanSetTimer);
  pFrame->payload.setTimer.speed = speed;
  pFrame->payload.setTimer.timer = timer;

  this->sendRfFrame(FAN_UNIT_TYPE_REMOTE, FAN_NETWORK_LINK_TTL);

  this->state_ = StateWaitSetSpeedConfirm;
}

void ZehnderRF::fanSettingsReceived(const uint8_t speed, const uint8_t voltage, const uint8_t timer) {
  ESP_LOGD(TAG, "Fan settings speed %u, voltage %u, timer %u", speed, voltage, timer);
}

void ZehnderRF::queryDevice(void) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;

  pFrame->command = FAN_UPDATE_SETTINGS;
  pFrame->parameter_count = 0;

  this->sendRfFrame(FAN_UNIT_TYPE_REMOTE, FAN_NETWORK_LINK_TTL);

  this->lastFanQuery_ = millis();  // Update time
  this->state_ = StateWaitQueryForUpdate;
}

uint8_t ZehnderRF::createDeviceID(void) {
  uint32_t v = 0;
  uint32_t v1, v2;

  v1 = fnv1_hash(esphome::get_name().c_str());
  v2 = fnv1_hash(esphome::get_mac_address().c_str());
  v = fnv1_hash(esphome::get_mac_address().c_str());

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

}  // namespace zehnder
}  // namespace esphome
