#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"


namespace esphome{
namespace climateguard{

class ClimateGuardAnemometerComponent : public PollingComponent, public i2c::I2CDevice {
 protected:
  enum ErrorCode {
    NONE,
    COMMUNICATION_FAILED,
    STATUS_FAILED,
    OVER_VOLTAGE,
  } error_code_{NONE};

  uint8_t chip_id_;
  uint8_t firmware_id_;

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *anemometer_sensor_{nullptr};
  sensor::Sensor *air_consumption_sensor_{nullptr};

 private:
  float duct_area_ = 0;

  struct SensorStatus {
    bool over_voltage = false;
    bool taring_error = false;
    bool watchdog = false;
    ErrorCode error;
  };

  SensorStatus get_sensor_status_();
  float get_data_from_registers_(uint8_t register_h, uint8_t register_l);
  float get_temperature_();
  float get_air_flow_rate_();
  float calculate_air_consumption_();

 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_anemometer_sensor(sensor::Sensor *anemometer_sensor) { anemometer_sensor_ = anemometer_sensor; }
  void set_air_consumption_sensor(sensor::Sensor *air_consumption_sensor) { air_consumption_sensor_ = air_consumption_sensor; }
};

}  // namespace climateguard
}  // namespace esphome