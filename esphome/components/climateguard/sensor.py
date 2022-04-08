import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_ANEMOMETER,
    CONF_FLOW_METER,
    CONF_TEMPERATURE,
    UNIT_CELSIUS,
    UNIT_CUBIC_METER_PER_HOUR,
    UNIT_METER_PER_SECOND,
    DEVICE_CLASS_ANEMOMETER,
    DEVICE_CLASS_FLOW_METER,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["i2c"]

climate_guard_ns = cg.esphome_ns("climateguard")
ClimateGuardAnemometerComponent = climate_guard_ns.class_(
    "ClimateGuardAnemometerComponent", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ClimateGuardAnemometerComponent),
            cv.Optional(CONF_ADDRESS): cv.hex_int,
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ANEMOMETER): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER_PER_SECOND,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_ANEMOMETER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FLOW_METER): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER_PER_HOUR,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_FLOW_METER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
        .extend(cv.polling_component_schema("60s"))
        .extend(i2c.i2c_device_schema(0x11))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_ANEMOMETER in config:
        sens = await sensor.new_sensor(config[CONF_ANEMOMETER])
        cg.add(var.set_anemometer_sensor(sens))

    if CONF_FLOW_METER in config:
        sens = await sensor.new_sensor(config[CONF_FLOW_METER])
        cg.add(var.set_air_consumption_sensor(sens))



