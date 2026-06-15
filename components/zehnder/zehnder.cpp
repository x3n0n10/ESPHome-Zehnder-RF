#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome
{
  namespace zehnder
  {

#define MAX_TRANSMIT_TIME 2000

    static const char *const TAG = "zehnder";

    // Fixed acknowledgement payload captured from a genuine remote, echoed back to
    // the main unit after it reports new fan settings. The exact meaning of these
    // bytes is unknown, but a real remote sends them verbatim.
    static const uint8_t FAN_SETTINGS_ACK_PAYLOAD[3] = {0x54, 0x03, 0x20};

    ZehnderRF::ZehnderRF(void) {}

    fan::FanTraits ZehnderRF::get_traits() { return fan::FanTraits(false, true, false, this->speed_count_); }

    void ZehnderRF::control(const fan::FanCall &call)
    {
      if (call.get_state().has_value())
      {
        this->state = *call.get_state();
        ESP_LOGD(TAG, "Control has state: %u", this->state);
      }
      if (call.get_speed().has_value())
      {
        this->speed = *call.get_speed();
        ESP_LOGD(TAG, "Control has speed: %u", this->speed);
      }

      switch (this->fsmState_)
      {
      case StateIdle:
        // Set speed
        this->setSpeed(this->state ? this->speed : 0x00, 0);

        this->lastFanQuery_ = millis(); // Update time
        break;

      default:
        break;
      }

      this->publish_state();
    }

    void ZehnderRF::setup()
    {
      ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

      // Clear config
      memset(&this->config_, 0, sizeof(Config));

      uint32_t hash = fnv1_hash("zehnderrf");
      this->pref_ = global_preferences->make_preference<Config>(hash, true);
      if (this->pref_.load(&this->config_))
      {
        ESP_LOGD(TAG, "Config load ok");
      }

      // Set nRF905 config
      nrf905::Config rfConfig;
      rfConfig = this->rf_->getConfig();

      rfConfig.band = true;
      rfConfig.channel = FAN_RF_CHANNEL;

      // CRC 16
      rfConfig.crc_enable = true;
      rfConfig.crc_bits = 16;

      // TX power
      rfConfig.tx_power = FAN_RF_TX_POWER;

      // RX power normal
      rfConfig.rx_power = nrf905::PowerNormal;

      rfConfig.rx_address = FAN_DEFAULT_RF_ADDRESS;
      rfConfig.rx_address_width = 4;
      rfConfig.rx_payload_width = 16;

      rfConfig.tx_address_width = 4;
      rfConfig.tx_payload_width = 16;

      rfConfig.xtal_frequency = 16000000; // defaults for now
      rfConfig.clkOutFrequency = nrf905::ClkOut500000;
      rfConfig.clkOutEnable = false;

      // Write config back
      this->rf_->updateConfig(&rfConfig);
      this->rf_->writeTxAddress(FAN_DEFAULT_RF_ADDRESS);

      this->speed_count_ = 4;

      this->lastFilterQuery_ = 0;
      this->lastErrorQuery_ = 0;
      this->lastStateIdleTime_ = millis();

      this->rf_->setOnTxReady([this](void)
                              {
    ESP_LOGD(TAG, "Tx Ready");
    if (this->rfState_ == RfStateTxBusy) {
      if (this->retries_ >= 0) {
        this->msgSendTime_ = millis();
        this->rfState_ = RfStateRxWait;
      } else {
        this->rfState_ = RfStateIdle;
      }
    } });

      this->rf_->setOnRxComplete([this](const uint8_t *const pData, const uint8_t dataLength)
                                 {
    ESP_LOGV(TAG, "Received frame");
    this->rfHandleReceived(pData, dataLength); });
    }

    void ZehnderRF::dump_config(void)
    {
      ESP_LOGCONFIG(TAG, "Zehnder Fan config:");
      ESP_LOGCONFIG(TAG, "  Component version  %s", ZEHNDER_RF_VERSION);
      ESP_LOGCONFIG(TAG, "  Polling interval   %u", this->interval_);
      ESP_LOGCONFIG(TAG, "  Fan networkId      0x%08X", this->config_.fan_networkId);
      ESP_LOGCONFIG(TAG, "  Fan my device type 0x%02X", this->config_.fan_my_device_type);
      ESP_LOGCONFIG(TAG, "  Fan my device id   0x%02X", this->config_.fan_my_device_id);
      ESP_LOGCONFIG(TAG, "  Fan main_unit type 0x%02X", this->config_.fan_main_unit_type);
      ESP_LOGCONFIG(TAG, "  Fan main unit id   0x%02X", this->config_.fan_main_unit_id);

      // Log diagnostic sensors
      if (this->filter_remaining_sensor_ != nullptr)
      {
        ESP_LOGCONFIG(TAG, "  Filter remaining sensor: YES");
      }
      if (this->filter_runtime_sensor_ != nullptr)
      {
        ESP_LOGCONFIG(TAG, "  Filter runtime sensor: YES");
      }
      if (this->error_count_sensor_ != nullptr)
      {
        ESP_LOGCONFIG(TAG, "  Error count sensor: YES");
      }
      if (this->error_code_sensor_ != nullptr)
      {
        ESP_LOGCONFIG(TAG, "  Error code sensor: YES");
      }
    }

    void ZehnderRF::set_config(const uint32_t fan_networkId,
                               const uint8_t  fan_my_device_type,
                               const uint8_t  fan_my_device_id,
                               const uint8_t  fan_main_unit_type,
                               const uint8_t  fan_main_unit_id) {
      this->config_.fan_networkId      = fan_networkId;      // Fan (Zehnder/BUVA) network ID
      this->config_.fan_my_device_type = fan_my_device_type; // Fan (Zehnder/BUVA) device type
      this->config_.fan_my_device_id   = fan_my_device_id;   // Fan (Zehnder/BUVA) device ID
      this->config_.fan_main_unit_type = fan_main_unit_type; // Fan (Zehnder/BUVA) main unit type
      this->config_.fan_main_unit_id   = fan_main_unit_id;   // Fan (Zehnder/BUVA) main unit ID
      ESP_LOGD(TAG, "Saving pairing config");
      this->pref_.save(&this->config_);
      global_preferences->sync();
    }

    void ZehnderRF::loop(void)
    {
      uint8_t deviceId;
      nrf905::Config rfConfig;

      // Run RF handler
      this->rfHandler();

      // State machine watchdog: if we get stuck waiting in any non-idle state
      // (other than startup/pairing, which legitimately wait for the user), force
      // a recovery back to idle. This guarantees the component self-heals instead
      // of needing a manual reboot.
      if (this->fsmState_ == StateIdle)
      {
        this->lastStateIdleTime_ = millis();
      }
      else if ((this->fsmState_ != StateStartup) && (this->fsmState_ != StateStartDiscovery) &&
               (this->fsmState_ != StateDiscoveryWaitForLinkRequest) &&
               (this->fsmState_ != StateDiscoveryWaitForJoinResponse) &&
               (this->fsmState_ != StateDiscoveryJoinComplete))
      {
        if ((millis() - this->lastStateIdleTime_) > FAN_STATE_WATCHDOG_TIMEOUT)
        {
          ESP_LOGW(TAG, "State machine stuck in state 0x%02X, forcing recovery", this->fsmState_);
          this->rfComplete();
          this->fsmState_ = StateIdle;
          this->lastStateIdleTime_ = millis();
        }
      }

      switch (this->fsmState_)
      {
      case StateStartup:
        // Wait until started up
        if (millis() > FAN_STARTUP_DELAY_MS)
        {
          // Discovery?
          if ((this->config_.fan_networkId == 0x00000000) || (this->config_.fan_my_device_type == 0) ||
              (this->config_.fan_my_device_id == 0) || (this->config_.fan_main_unit_type == 0) ||
              (this->config_.fan_main_unit_id == 0))
          {
            ESP_LOGD(TAG, "Invalid config, start paring");

            this->fsmState_ = StateStartDiscovery;
          }
          else
          {
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
        if (this->rfState_ == RfStateIdle)
        {
          // When done, return to idle
          this->fsmState_ = StateIdle;
        }
        break;

      case StateIdle:
        if (newSetting == true)
        {
          this->setSpeed(newSpeed, newTimer);
        }
        else
        {
          // Existing fan query
          if ((millis() - this->lastFanQuery_) > this->interval_)
          {
            this->queryDevice();
          }

          // Add filter query every 10 minutes if sensors are connected
          if (((millis() - this->lastFilterQuery_) > FAN_FILTER_QUERY_INTERVAL_MS) &&
              (this->filter_remaining_sensor_ != nullptr || this->filter_runtime_sensor_ != nullptr))
          {
            this->queryFilterStatus();
          }

          // Add error query every 5 minutes if sensors are connected
          if (((millis() - this->lastErrorQuery_) > FAN_ERROR_QUERY_INTERVAL_MS) &&
              (this->error_count_sensor_ != nullptr || this->error_code_sensor_ != nullptr))
          {
            this->queryErrorStatus();
          }
        }
        break;

      default:
        break;
      }
    }

    void ZehnderRF::queryErrorStatus(void)
    {
      RfFrame *const pFrame = (RfFrame *)this->_txFrame; // frame helper

      ESP_LOGD(TAG, "Query error status");

      this->lastErrorQuery_ = millis(); // Update time

      // Clear frame data
      (void)memset(this->_txFrame, 0, FAN_FRAMESIZE);

      // Build frame
      pFrame->rx_type = this->config_.fan_main_unit_type;
      pFrame->rx_id = this->config_.fan_main_unit_id;
      pFrame->tx_type = this->config_.fan_my_device_type;
      pFrame->tx_id = this->config_.fan_my_device_id;
      pFrame->ttl = FAN_TTL;
      pFrame->command = FAN_TYPE_QUERY_ERROR_STATUS;
      pFrame->parameter_count = 0x00; // No parameters

      this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]()
                          {
    ESP_LOGW(TAG, "Error status query timeout");
    this->fsmState_ = StateIdle; });

      this->fsmState_ = StateWaitErrorStatusResponse;
    }

    void ZehnderRF::queryFilterStatus(void)
    {
      RfFrame *const pFrame = (RfFrame *)this->_txFrame; // frame helper

      ESP_LOGD(TAG, "Query filter status");

      this->lastFilterQuery_ = millis(); // Update time

      // Clear frame data
      (void)memset(this->_txFrame, 0, FAN_FRAMESIZE);

      // Build frame
      pFrame->rx_type = this->config_.fan_main_unit_type;
      pFrame->rx_id = this->config_.fan_main_unit_id;
      pFrame->tx_type = this->config_.fan_my_device_type;
      pFrame->tx_id = this->config_.fan_my_device_id;
      pFrame->ttl = FAN_TTL;
      pFrame->command = FAN_TYPE_QUERY_FILTER_STATUS;
      pFrame->parameter_count = 0x00; // No parameters

      this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]()
                          {
    ESP_LOGW(TAG, "Filter status query timeout");
    this->fsmState_ = StateIdle; });

      this->fsmState_ = StateWaitFilterStatusResponse;
    }

    // Dispatches a received frame to the handler for the current protocol state.
    void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength)
    {
      const RfFrame *const pResponse = (const RfFrame *)pData;

      ESP_LOGD(TAG, "Current state: 0x%02X", this->fsmState_);
      switch (this->fsmState_)
      {
      case StateDiscoveryWaitForLinkRequest:
        this->rfHandleLinkRequest(pResponse);
        break;

      case StateDiscoveryWaitForJoinResponse:
        this->rfHandleJoinResponse(pResponse);
        break;

      case StateDiscoveryJoinComplete:
        this->rfHandleJoinComplete(pResponse);
        break;

      case StateIdle:
      case StateWaitQueryResponse:
        // Both states accept an unsolicited / polled fan-settings update.
        this->rfHandleFanSettingsResponse(pResponse);
        break;

      case StateWaitFilterStatusResponse:
        this->rfHandleFilterStatusResponse(pResponse);
        break;

      case StateWaitErrorStatusResponse:
        this->rfHandleErrorStatusResponse(pResponse);
        break;

      case StateWaitSetSpeedResponse:
        this->rfHandleSetSpeedResponse(pResponse);
        break;

      default:
        ESP_LOGD(TAG, "Received frame from unknown device in unknown state; type 0x%02X from ID 0x%02X type 0x%02X",
                 pResponse->command, pResponse->tx_id, pResponse->tx_type);
        break;
      }
    }

    bool ZehnderRF::addressedToUs(const RfFrame *const pResponse) const
    {
      return (pResponse->rx_type == this->config_.fan_my_device_type) &&
             (pResponse->rx_id == this->config_.fan_my_device_id);
    }

    // Applies a fan-settings (0x07) frame to our published state.
    void ZehnderRF::handleFanSettings(const RfFrame *const pResponse)
    {
      ESP_LOGD(TAG, "Received fan settings; speed: 0x%02X voltage: %i timer: %i",
               pResponse->payload.fanSettings.speed, pResponse->payload.fanSettings.voltage,
               pResponse->payload.fanSettings.timer);

      this->rfComplete();

      this->state = pResponse->payload.fanSettings.speed > 0;
      this->speed = pResponse->payload.fanSettings.speed;
      this->timer = pResponse->payload.fanSettings.timer;
      this->voltage = pResponse->payload.fanSettings.voltage;
      this->publish_state();
    }

    // Pairing: main unit opened its network, so request to join it.
    void ZehnderRF::rfHandleLinkRequest(const RfFrame *const pResponse)
    {
      RfFrame *const pTxFrame = (RfFrame *)this->_txFrame; // frame helper
      nrf905::Config rfConfig;

      ESP_LOGD(TAG, "DiscoverStateWaitForLinkRequest");
      switch (pResponse->command)
      {
      case FAN_NETWORK_JOIN_OPEN: // Received linking request from main unit
        ESP_LOGD(TAG, "Discovery: Found unit type 0x%02X (%s) with ID 0x%02X on network 0x%08X", pResponse->tx_type,
                 pResponse->tx_type == FAN_TYPE_MAIN_UNIT ? "Main" : "?", pResponse->tx_id,
                 pResponse->payload.networkJoinOpen.networkId);

        this->rfComplete();

        (void)memset(this->_txFrame, 0, FAN_FRAMESIZE); // Clear frame data

        // Found a main unit, so send a join request
        pTxFrame->rx_type = FAN_TYPE_MAIN_UNIT; // Set type to main unit
        pTxFrame->rx_id = pResponse->tx_id;     // Set ID to the ID of the main unit
        pTxFrame->tx_type = this->config_.fan_my_device_type;
        pTxFrame->tx_id = this->config_.fan_my_device_id;
        pTxFrame->ttl = FAN_TTL;
        pTxFrame->command = FAN_NETWORK_JOIN_REQUEST; // Request to connect to network
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
        this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]()
                            {
          ESP_LOGW(TAG, "Query Timeout");
          this->fsmState_ = StateStartDiscovery; });

        this->fsmState_ = StateDiscoveryWaitForJoinResponse;
        break;

      default:
        ESP_LOGD(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X", pResponse->command,
                 pResponse->tx_id);
        break;
      }
    }

    // Pairing: main unit acknowledged our join request, so confirm the link.
    void ZehnderRF::rfHandleJoinResponse(const RfFrame *const pResponse)
    {
      RfFrame *const pTxFrame = (RfFrame *)this->_txFrame; // frame helper

      ESP_LOGD(TAG, "DiscoverStateWaitForJoinResponse");
      switch (pResponse->command)
      {
      case FAN_FRAME_0B:
        if ((pResponse->rx_type == this->config_.fan_my_device_type) &&
            (pResponse->rx_id == this->config_.fan_my_device_id) &&
            (pResponse->tx_type == this->config_.fan_main_unit_type) &&
            (pResponse->tx_id == this->config_.fan_main_unit_id))
        {
          ESP_LOGD(TAG, "Discovery: Link successful to unit with ID 0x%02X on network 0x%08X", pResponse->tx_id,
                   this->config_.fan_networkId);

          this->rfComplete();

          (void)memset(this->_txFrame, 0, FAN_FRAMESIZE); // Clear frame data

          pTxFrame->rx_type = FAN_TYPE_MAIN_UNIT; // Set type to main unit
          pTxFrame->rx_id = pResponse->tx_id;     // Set ID to the ID of the main unit
          pTxFrame->tx_type = this->config_.fan_my_device_type;
          pTxFrame->tx_id = this->config_.fan_my_device_id;
          pTxFrame->ttl = FAN_TTL;
          pTxFrame->command = FAN_FRAME_0B; // 0x0B acknowledge link successful
          pTxFrame->parameter_count = 0x00; // No parameters

          // Send response frame
          this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]()
                              {
            ESP_LOGW(TAG, "Query Timeout");
            this->fsmState_ = StateStartDiscovery; });

          this->fsmState_ = StateDiscoveryJoinComplete;
        }
        else
        {
          ESP_LOGE(TAG, "Discovery: Received unknown link success from ID 0x%02X on network 0x%08X", pResponse->tx_id,
                   this->config_.fan_networkId);
        }
        break;

      default:
        ESP_LOGE(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X", pResponse->command,
                 pResponse->tx_id);
        break;
      }
    }

    // Pairing: main unit confirmed the network join; persist config and go idle.
    void ZehnderRF::rfHandleJoinComplete(const RfFrame *const pResponse)
    {
      ESP_LOGD(TAG, "StateDiscoveryJoinComplete");
      switch (pResponse->command)
      {
      case FAN_TYPE_QUERY_NETWORK:
        if ((pResponse->rx_type == this->config_.fan_main_unit_type) &&
            (pResponse->rx_id == this->config_.fan_main_unit_id) &&
            (pResponse->tx_type == this->config_.fan_main_unit_type) &&
            (pResponse->tx_id == this->config_.fan_main_unit_id))
        {
          ESP_LOGD(TAG, "Discovery: received network join success 0x0D");

          this->rfComplete();

          ESP_LOGD(TAG, "Saving pairing config");
          this->pref_.save(&this->config_);
          // Force the pairing data to flash immediately. Otherwise it only gets
          // written on the (long) flash_write_interval, so a power cycle before
          // then loses the pairing and forces the user to pair again.
          global_preferences->sync();

          this->fsmState_ = StateIdle;
        }
        else
        {
          ESP_LOGW(TAG, "Unexpected frame join reponse from Type 0x%02X ID 0x%02X", pResponse->tx_type,
                   pResponse->tx_id);
        }
        break;

      default:
        ESP_LOGE(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X on network 0x%08X",
                 pResponse->command, pResponse->tx_id, this->config_.fan_networkId);
        break;
      }
    }

    // Idle / query response: the fan reported its current settings.
    void ZehnderRF::rfHandleFanSettingsResponse(const RfFrame *const pResponse)
    {
      if (!this->addressedToUs(pResponse))
      {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X", pResponse->command,
                 pResponse->tx_id, pResponse->tx_type);
        return;
      }

      switch (pResponse->command)
      {
      case FAN_TYPE_FAN_SETTINGS:
        this->handleFanSettings(pResponse);
        this->fsmState_ = StateIdle;
        break;

      default:
        ESP_LOGD(TAG, "Received unexpected frame; type 0x%02X from ID 0x%02X", pResponse->command, pResponse->tx_id);
        break;
      }
    }

    void ZehnderRF::rfHandleFilterStatusResponse(const RfFrame *const pResponse)
    {
      if (!this->addressedToUs(pResponse))
      {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X", pResponse->command,
                 pResponse->tx_id, pResponse->tx_type);
        return;
      }

      switch (pResponse->command)
      {
      case FAN_TYPE_FILTER_STATUS_RESPONSE:
      {
        const RfPayloadFilterStatus *filterStatus =
            reinterpret_cast<const RfPayloadFilterStatus *>(&pResponse->payload);

        ESP_LOGD(TAG, "Received filter status; total hours: %u, filter hours: %u, remaining: %u%%",
                 filterStatus->totalRunHours, filterStatus->filterRunHours, filterStatus->filterPercentRemaining);

        this->rfComplete();

        // Update sensor values if you've added them
        if (this->filter_remaining_sensor_ != nullptr)
        {
          this->filter_remaining_sensor_->publish_state(filterStatus->filterPercentRemaining);
        }
        if (this->filter_runtime_sensor_ != nullptr)
        {
          this->filter_runtime_sensor_->publish_state(filterStatus->filterRunHours);
        }

        this->fsmState_ = StateIdle;
        break;
      }

      default:
        ESP_LOGD(TAG, "Received unexpected frame; type 0x%02X from ID 0x%02X", pResponse->command, pResponse->tx_id);
        break;
      }
    }

    void ZehnderRF::rfHandleErrorStatusResponse(const RfFrame *const pResponse)
    {
      if (!this->addressedToUs(pResponse))
      {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X", pResponse->command,
                 pResponse->tx_id, pResponse->tx_type);
        return;
      }

      switch (pResponse->command)
      {
      case FAN_TYPE_ERROR_STATUS_RESPONSE:
      {
        const RfPayloadErrorStatus *errorStatus =
            reinterpret_cast<const RfPayloadErrorStatus *>(&pResponse->payload);

        ESP_LOGD(TAG, "Received error status; count: %u, severity: %u", errorStatus->errorCount,
                 errorStatus->errorSeverity);

        this->rfComplete();

        // Update sensor values
        if (this->error_count_sensor_ != nullptr)
        {
          this->error_count_sensor_->publish_state(errorStatus->errorCount);
        }

        if (this->error_code_sensor_ != nullptr && errorStatus->errorCount > 0)
        {
          char error_text[32];
          snprintf(error_text, sizeof(error_text), "E%02d", errorStatus->errorCodes[0]);
          for (int i = 1; i < errorStatus->errorCount && i < 5; i++)
          {
            char temp[8];
            snprintf(temp, sizeof(temp), ",E%02d", errorStatus->errorCodes[i]);
            strncat(error_text, temp, sizeof(error_text) - strlen(error_text) - 1);
          }
          this->error_code_sensor_->publish_state(error_text);
        }
        else if (this->error_code_sensor_ != nullptr)
        {
          this->error_code_sensor_->publish_state("No Errors");
        }

        this->fsmState_ = StateIdle;
        break;
      }

      default:
        ESP_LOGD(TAG, "Received unexpected frame; type 0x%02X from ID 0x%02X", pResponse->command, pResponse->tx_id);
        break;
      }
    }

    // Set-speed response: fan reported new settings, so ack and wait for confirm.
    void ZehnderRF::rfHandleSetSpeedResponse(const RfFrame *const pResponse)
    {
      RfFrame *const pTxFrame = (RfFrame *)this->_txFrame; // frame helper

      if (!this->addressedToUs(pResponse))
      {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X", pResponse->command,
                 pResponse->tx_id, pResponse->tx_type);
        return;
      }

      switch (pResponse->command)
      {
      case FAN_TYPE_FAN_SETTINGS:
        this->handleFanSettings(pResponse);

        (void)memset(this->_txFrame, 0, FAN_FRAMESIZE); // Clear frame data

        pTxFrame->rx_type = this->config_.fan_main_unit_type; // Set type to main unit
        pTxFrame->rx_id = this->config_.fan_main_unit_id;     // Set ID to the ID of the main unit
        pTxFrame->tx_type = this->config_.fan_my_device_type;
        pTxFrame->tx_id = this->config_.fan_my_device_id;
        pTxFrame->ttl = FAN_TTL;
        pTxFrame->command = FAN_FRAME_SETSPEED_REPLY;
        pTxFrame->parameter_count = sizeof(FAN_SETTINGS_ACK_PAYLOAD); // 3 parameters
        (void)memcpy(pTxFrame->payload.parameters, FAN_SETTINGS_ACK_PAYLOAD, sizeof(FAN_SETTINGS_ACK_PAYLOAD));

        // Send response frame
        this->startTransmit(this->_txFrame, -1, NULL);

        this->fsmState_ = StateWaitSetSpeedConfirm;
        break;

      case FAN_FRAME_SETSPEED_REPLY:
      case FAN_FRAME_SETVOLTAGE_REPLY:
        // Acknowledgement of our own reply; nothing further to do here.
        break;

      default:
        ESP_LOGD(TAG, "Received unexpected frame; type 0x%02X from ID 0x%02X", pResponse->command, pResponse->tx_id);
        break;
      }
    }

    static uint8_t minmax(const uint8_t value, const uint8_t min, const uint8_t max)
    {
      if (value <= min)
      {
        return min;
      }
      else if (value >= max)
      {
        return max;
      }
      else
      {
        return value;
      }
    }

    uint8_t ZehnderRF::createDeviceID(void)
    {
      uint8_t random = (uint8_t)random_uint32();
      // Generate random device_id; don't use 0x00 and 0xFF

      // TODO: there's a 1 in 255 chance that the generated ID matches the ID of the main unit. Decide how to deal
      // withthis (some sort of ping discovery?)

      return minmax(random, 1, 0xFE);
    }

    void ZehnderRF::queryDevice(void)
    {
      RfFrame *const pFrame = (RfFrame *)this->_txFrame; // frame helper

      ESP_LOGD(TAG, "Query device");

      this->lastFanQuery_ = millis(); // Update time

      // Clear frame data
      (void)memset(this->_txFrame, 0, FAN_FRAMESIZE);

      // Build frame
      pFrame->rx_type = this->config_.fan_main_unit_type;
      pFrame->rx_id = this->config_.fan_main_unit_id;
      pFrame->tx_type = this->config_.fan_my_device_type;
      pFrame->tx_id = this->config_.fan_my_device_id;
      pFrame->ttl = FAN_TTL;
      pFrame->command = FAN_TYPE_QUERY_DEVICE;
      pFrame->parameter_count = 0x00; // No parameters

      this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]()
                          {
    ESP_LOGW(TAG, "Query Timeout");
    this->fsmState_ = StateIdle; });

      this->fsmState_ = StateWaitQueryResponse;
    }

    void ZehnderRF::setSpeed(const uint8_t paramSpeed, const uint8_t paramTimer)
    {
      RfFrame *const pFrame = (RfFrame *)this->_txFrame; // frame helper
      uint8_t speed = paramSpeed;
      uint8_t timer = paramTimer;

      if (speed > this->speed_count_)
      {
        ESP_LOGW(TAG, "Requested speed too high (%u)", speed);
        speed = this->speed_count_;
      }

      ESP_LOGD(TAG, "Set speed: 0x%02X; Timer %u minutes", speed, timer);

      if (this->fsmState_ == StateIdle)
      {
        (void)memset(this->_txFrame, 0, FAN_FRAMESIZE); // Clear frame data

        // Build frame
        pFrame->rx_type = this->config_.fan_main_unit_type;
        pFrame->rx_id = 0x00; // Broadcast
        pFrame->tx_type = this->config_.fan_my_device_type;
        pFrame->tx_id = this->config_.fan_my_device_id;
        pFrame->ttl = FAN_TTL;

        if (timer == 0)
        {
          pFrame->command = FAN_FRAME_SETSPEED;
          pFrame->parameter_count = sizeof(RfPayloadFanSetSpeed);
          pFrame->payload.setSpeed.speed = speed;
        }
        else
        {
          pFrame->command = FAN_FRAME_SETTIMER;
          pFrame->parameter_count = sizeof(RfPayloadFanSetTimer);
          pFrame->payload.setTimer.speed = speed;
          pFrame->payload.setTimer.timer = timer;
        }

        ESP_LOGD(TAG, "Waiting for initial RF stabilization...");
        delay(500); // Wait 500ms for RF to stabilize

        this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]()
                            {
      ESP_LOGW(TAG, "Set speed timeout");
      this->fsmState_ = StateIdle; });

        newSetting = false;
        this->fsmState_ = StateWaitSetSpeedResponse;
      }
      else
      {
        ESP_LOGD(TAG, "Invalid state, I'm trying later again");
        newSpeed = speed;
        newTimer = timer;
        newSetting = true;
      }
    }

    void ZehnderRF::discoveryStart(const uint8_t deviceId)
    {
      RfFrame *const pFrame = (RfFrame *)this->_txFrame; // frame helper
      nrf905::Config rfConfig;

      ESP_LOGD(TAG, "Start discovery with ID %u", deviceId);

      this->config_.fan_my_device_type = FAN_TYPE_REMOTE_CONTROL;
      this->config_.fan_my_device_id = deviceId;

      // Build frame
      (void)memset(this->_txFrame, 0, FAN_FRAMESIZE); // Clear frame data

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

      this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]()
                          {
    ESP_LOGW(TAG, "Start discovery timeout");
    this->fsmState_ = StateStartDiscovery; });

      // Update state
      this->fsmState_ = StateDiscoveryWaitForLinkRequest;
    }

    Result ZehnderRF::startTransmit(const uint8_t *const pData, const int8_t rxRetries,
                                    const std::function<void(void)> callback)
    {
      Result result = ResultOk;

      if (this->rfState_ != RfStateIdle)
      {
        ESP_LOGW(TAG, "TX still ongoing");
        result = ResultBusy;
      }
      else
      {
        this->onReceiveTimeout_ = callback;
        this->retries_ = rxRetries;

        // Load the frame into the nRF905, then wait for a free airway before TX.
        this->rf_->writeTxPayload(pData, FAN_FRAMESIZE); // Use framesize

        this->rfState_ = RfStateWaitAirwayFree;
        this->airwayFreeWaitTime_ = millis();
      }

      return result;
    }

    void ZehnderRF::rfComplete(void)
    {
      this->retries_ = -1; // Disable this->retries_
      this->rfState_ = RfStateIdle;
    }

    void ZehnderRF::rfHandler(void)
    {
      switch (this->rfState_)
      {
      case RfStateIdle:
        break;

      case RfStateWaitAirwayFree:
        if ((millis() - this->airwayFreeWaitTime_) > FAN_AIRWAY_TIMEOUT_MS)
        {
          ESP_LOGW(TAG, "Airway too busy, giving up");
          this->rfState_ = RfStateIdle;

          if (this->onReceiveTimeout_ != NULL)
          {
            this->onReceiveTimeout_();
          }
        }
        else if (this->rf_->airwayBusy() == false)
        {
          ESP_LOGD(TAG, "Start TX");
          this->rf_->startTx(FAN_TX_FRAMES, nrf905::Receive); // After transmit, wait for response

          this->txStartTime_ = millis();
          this->rfState_ = RfStateTxBusy;
        }
        break;

      case RfStateTxBusy:
        // Safety net: the transition out of TxBusy depends on the nRF905 TxReady
        // callback, which is driven by polling the DR status bit. If that edge is
        // ever missed the radio would otherwise stay busy forever and require a
        // reboot. Time out and reset instead.
        if ((millis() - this->txStartTime_) > FAN_TX_TIMEOUT)
        {
          ESP_LOGW(TAG, "TX timeout (no TxReady), resetting radio");
          this->rfState_ = RfStateIdle;

          if (this->onReceiveTimeout_ != NULL)
          {
            this->onReceiveTimeout_();
          }
        }
        break;

      case RfStateRxWait:
        if ((this->retries_ >= 0) && ((millis() - this->msgSendTime_) > FAN_REPLY_TIMEOUT))
        {
          ESP_LOGD(TAG, "Receive timeout");

          if (this->retries_ > 0)
          {
            --this->retries_;
            ESP_LOGD(TAG, "No data received, retry again (left: %u)", this->retries_);

            // Non-blocking pause between retries (handled in RfStateRetryWait)
            this->retryWaitTime_ = millis();
            this->rfState_ = RfStateRetryWait;
          }
          else if (this->retries_ == 0)
          {
            // Oh oh, ran out of options

            ESP_LOGD(TAG, "No messages received, giving up now...");
            if (this->onReceiveTimeout_ != NULL)
            {
              this->onReceiveTimeout_();
            }

            // Back to idle
            this->rfState_ = RfStateIdle;
          }
        }
        break;

      case RfStateRetryWait:
        // Wait (without blocking the main loop) before re-attempting transmit
        if ((millis() - this->retryWaitTime_) > FAN_RETRY_DELAY)
        {
          this->rfState_ = RfStateWaitAirwayFree;
          this->airwayFreeWaitTime_ = millis();
        }
        break;

      default:
        break;
      }
    }

  } // namespace zehnder
} // namespace esphome
