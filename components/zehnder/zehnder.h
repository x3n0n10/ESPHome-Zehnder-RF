#pragma once

#include "esphome/core/component.h"
#include "esphome/components/fan/fan.h"

namespace esphome {
namespace zehnder {

class ZehnderRF : public Component, public fan::Fan {
 public:
  ZehnderRF(void);

  // Fan implementation
  fan::FanTraits get_traits() override;
  void control(const fan::FanCall &call) override;

  void setup() override;
  void loop() override;
  void dump_config() override;

  // Additional methods
  void discoveryStart(uint8_t deviceId); // Correct parameter type

 protected:
  void rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength);
  void rfHandler(void);
  void sendRfFrame(const uint8_t deviceType, const uint8_t ttl);
  void setSpeed(const uint8_t speed, const uint8_t timer);
  void fanSettingsReceived(const uint8_t speed, const uint8_t voltage, const uint8_t timer);
  void queryDevice(void);
  uint8_t createDeviceID(void);
  uint16_t calculate_crc16(uint8_t *data, size_t length);
  void append_crc_to_payload(uint8_t *payload, size_t length);

 private:
  uint8_t _txFrame[32];
  uint8_t state_;
  uint8_t speed_;
  uint8_t lastFanQuery_;
  uint8_t speed_count_;
  uint32_t interval_;
  uint32_t msgSendTime_;
  uint8_t retries_;
  uint8_t rfState_;
  uint8_t config_;
  uint8_t fan_networkId_;
  uint8_t fan_my_device_type_;
  uint8_t fan_my_device_id_;
  uint8_t fan_main_unit_type_;
  uint8_t fan_main_unit_id_;
  uint8_t fan_settings_;
  uint8_t fan_timer_;

  // RF frame related constants
  static constexpr uint8_t FAN_NETWORK_JOIN_OPEN = 0x01;
  static constexpr uint8_t FAN_NETWORK_JOIN_REQUEST = 0x02;
  static constexpr uint8_t FAN_NETWORK_JOIN_ACK = 0x03;
  static constexpr uint8_t FAN_SET_SPEED_TIMER = 0x04;
  static constexpr uint8_t FAN_UPDATE_SETTINGS = 0x05;
  static constexpr uint8_t FAN_TYPE_REMOTE = 0x10;
  static constexpr uint8_t FAN_TYPE_MAIN_UNIT = 0x11;
  static constexpr uint8_t FAN_TYPE_BROADCAST = 0xFF;
  static constexpr uint8_t FAN_TTL = 0xFF;

  enum {
    StateStartup,
    StateStartDiscovery,
    StateIdle,
    StateWaitSetSpeedConfirm,
    StateWaitQueryForUpdate,
    StateDiscoveryWaitForLinkRequest,
    StateDiscoveryWaitForLinkAck
  } state_;

  enum {
    RfStateIdle,
    RfStateTxBusy,
    RfStateRxWait,
    RfStateRxBusy
  } rfState_;
};

}  // namespace zehnder
}  // namespace esphome
