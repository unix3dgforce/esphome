#include "anemometer.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome{
namespace climateguard {

static const char *const TAG = "CG-Anemometer";

static const uint8_t CG_ANEM_FACTORY_ID_1 = 0x0;
static const uint8_t CG_ANEM_FACTORY_ID_2 = 0x1;
static const uint8_t CG_ANEM_FACTORY_ID_3 = 0x2;
static const uint8_t CG_ANEM_FACTORY_ID_4 = 0x3;
static const uint8_t CG_ANEM_VERSION = 0x4;
static const uint8_t CG_ANEM_WHO_I_AM = 0x5;
static const uint8_t CG_ANEM_STATUS = 0x6;
static const uint8_t CG_ANEM_WIND_H = 0x7;
static const uint8_t CG_ANEM_WIND_L = 0x8;

static const uint8_t CG_ANEM_ADC_COLD_H = 0x09;
static const uint8_t CG_ANEM_ADC_COLD_L = 0x0A;
static const uint8_t CG_ANEM_ADC_HOT_H = 0x0B;
static const uint8_t CG_ANEM_ADC_HOT_L = 0x0C;
static const uint8_t CG_ANEM_SUPPLY_V = 0x0D;
static const uint8_t CG_ANEM_PWR_WT = 0x0E;

static const uint8_t CG_ANEM_TEMP_COLD_H = 0x10;
static const uint8_t CG_ANEM_TEMP_COLD_L = 0x11;
static const uint8_t CG_ANEM_TEMP_HOT_H = 0x12;
static const uint8_t CG_ANEM_TEMP_HOT_L = 0x13;

static const uint8_t CG_ANEM_DT_H = 0x14;
static const uint8_t CG_ANEM_DT_L = 0x15;

void ClimateGuardAnemometerComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ClimateGuardAnemometer...");

  delay(1000);
  if (!this->read_byte(CG_ANEM_WHO_I_AM, &this->chip_id_)){
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  this->read_byte(CG_ANEM_VERSION, &this->firmware_id_);

}
void ClimateGuardAnemometerComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "ClimateGuardAnemometer:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);

  ESP_LOGCONFIG(TAG, "  Chip ID: 0x%x", this->chip_id_);
  ESP_LOGCONFIG(TAG, "  Firmware ID: 0x%x", this->firmware_id_);

  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Air flow rate", this->anemometer_sensor_);
  LOG_SENSOR("  ", "Air Consumption", this->air_consumption_sensor_);

  switch (this->error_code_) {
    case COMMUNICATION_FAILED:
      ESP_LOGCONFIG(TAG, "Connected device not match");
      break;
    case STATUS_FAILED:
      ESP_LOGCONFIG(TAG, "Sensor wiring error");
      break;
    case OVER_VOLTAGE:
      ESP_LOGCONFIG(TAG, "Input voltage is too high and module goes to defence");
      break;
    case NONE:
    default:
      break;
  }
}
void ClimateGuardAnemometerComponent::update() {
  SensorStatus sensor_status = this->get_sensor_status_();
  if (sensor_status.error != NONE || sensor_status.over_voltage)
  {
    if (sensor_status.over_voltage) {
      ESP_LOGE(TAG, "Input voltage is too high and module goes to defence");
    } else {
      ESP_LOGE(TAG, "Sensor wiring error");
    }
    return;
  }

  if (this->temperature_sensor_ != nullptr){
    this->temperature_sensor_->publish_state(this->get_temperature_());
  }

  if (this->anemometer_sensor_ != nullptr){
    this->anemometer_sensor_->publish_state(this->get_air_flow_rate_());
  }

  if (this->air_consumption_sensor_ != nullptr){
    this->anemometer_sensor_->publish_state(this->calculate_air_consumption_());
  }

}

float ClimateGuardAnemometerComponent::get_setup_priority() const { return setup_priority::BUS; }

// Get data from status register
ClimateGuardAnemometerComponent::SensorStatus ClimateGuardAnemometerComponent::get_sensor_status_() {
  uint8_t status_reg;
  SensorStatus sensor_status = {false,false,false,STATUS_FAILED};

  if(this->read_byte(CG_ANEM_STATUS, &status_reg)){
    sensor_status = {static_cast<bool>(status_reg & (1 << 0x1)),
                     static_cast<bool>((status_reg & (1 << 0x7)) | (status_reg & (1 << 0x6))),
                     static_cast<bool>(status_reg & (1 << 0x5)),
                     NONE
    };
  }

  if (sensor_status.over_voltage) { sensor_status.error = OVER_VOLTAGE; }

  return sensor_status;
}

// Get data
float ClimateGuardAnemometerComponent::get_data_from_registers_(uint8_t register_h, uint8_t register_l) {
  uint8_t raw[2];

  if (this->read_byte(register_h, &raw[0])) {
    if (this->read_byte(register_l, &raw[1])) {
      return ((raw[0] << 8) | raw[1]) / 10.0;
    }
  }
  return -255;
}

// Get current temperature
float ClimateGuardAnemometerComponent::get_temperature_() {
  return this->get_data_from_registers_(CG_ANEM_TEMP_COLD_H, CG_ANEM_TEMP_COLD_L);
}

// Get current flow rate
float ClimateGuardAnemometerComponent::get_air_flow_rate_() {
  return this->get_data_from_registers_(CG_ANEM_WIND_H, CG_ANEM_WIND_L);
}

//Calculate flow consumption
float ClimateGuardAnemometerComponent::calculate_air_consumption_() {
  float air_flow_rate = this->get_air_flow_rate_();
  if (duct_area_ > -0.01 && air_flow_rate != -255)
  {
    return 6 * duct_area_ *air_flow_rate * 0.06;
  }

  return -255;
}

}  // namespace climateguard
}  // namespace esphome
