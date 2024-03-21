#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace custom_phec_sensor {

class PhEcSensorComponent : public PollingComponent {
 public:
  // Конструктор класса: принимает адрес I2C датчика и интервал опроса
  PhEcSensorComponent(uint8_t address, uint32_t update_interval) : PollingComponent(update_interval), address_(address) {}

  // Указатели на объекты сенсоров для pH и температуры
  sensor::Sensor *ph_sensor{nullptr};
  sensor::Sensor *temperature_sensor{nullptr};

  float get_setup_priority() const override { return setup_priority::DATA; }

  void setup() override {
    // Здесь можно инициализировать I2C, если это не было сделано ранее
  }

  void update() override {
    read_module();
  }

 protected:
  uint8_t address_; // I2C адрес датчика

  // Метод чтения данных с модуля
  void read_module() {
    uint8_t buff[20];
    memset(buff, 0, sizeof(buff));

    Wire.beginTransmission(address_);
    Wire.requestFrom(int(address_), 20);
    if(Wire.readBytes(buff, sizeof(buff)) == 20) {
      process_sensor_data(buff);
    } else {
      ESP_LOGE(TAG, "Failed to read data from sensor.");
    }
    Wire.endTransmission();
  }

  // Метод обработки данных с датчика
  void process_sensor_data(const uint8_t *data) {
    if (data[0] != 0x50 || data[1] != 0x01) {
      ESP_LOGW(TAG, "Unexpected data header: %X %X", data[0], data[1]);
      return;
    }

    float ph = convert_to_ph(data);
    float temperature = convert_to_temperature(data);
    if (ph_sensor != nullptr) {
      ph_sensor->publish_state(ph);
    }
    if (temperature_sensor != nullptr) {
      temperature_sensor->publish_state(temperature);
    }
  }

  // Функция конвертации в pH
  float convert_to_ph(const uint8_t *data) {
    uint16_t ph100 = ((uint16_t)data[3] << 8) | data[2];
    return ph100 / 100.0f;
  }

  // Функция конвертации в температуру
  float convert_to_temperature(const uint8_t *data) {
    uint16_t temp100 = ((uint16_t)data[17] << 8) | data[16];
    if (temp100 <= 100) {
      return -1.0; // Значение "-1" может использоваться для индикации ошибки чтения датчика
    }
    return temp100 / 100.0f;
  }

  static const char *TAG;
};

const char *PhEcSensorComponent::TAG = "ph_ec_sensor";
}  // namespace custom_phec_sensor
}  // namespace esphome

