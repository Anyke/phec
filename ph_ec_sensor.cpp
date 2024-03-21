#include "ph_ec_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ph_ec_sensor {

static const char *const TAG = "ph_ec_sensor";

void PhEcSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PhEc Sensor...");
  Wire.begin(21, 22, 80000); // Инициализация I2C с указанием SDA = 4, SCL = 16 и скоростью 80000
}

void PhEcSensor::update() {
  read_module();
}

void PhEcSensor::read_module() {
  uint8_t buff[20];
  memset(buff, 0, 20); 

  Wire.beginTransmission(phAddr_);
  Wire.requestFrom(uint8_t(phAddr_), uint8_t(20));
  size_t length = Wire.readBytes(buff, sizeof(buff));
  Wire.endTransmission();

  if (length == 20 && buff[0] == 0x50 && buff[1] == 0x01) { // Проверка сигнатуры модуля и корректности данных
    process_sensor_data(buff);
  } else {
    ESP_LOGE(TAG, "Failed to read data correctly from sensor");
  }
}

void PhEcSensor::process_sensor_data(const uint8_t *data) {
    int16_t raw_u = ((int16_t)data[5] << 8 | data[4]) / 10;
    uint16_t ph100 = ((uint16_t)data[3] << 8) | data[2];
    float ph = ph100 / 100.0f;

    // Пример для отладки - отображение считанных значений
    ESP_LOGD(TAG, "Raw Voltage: %d, pH Value: %.2f", raw_u, ph);

    if (pH_sensor_) pH_sensor_->publish_state(ph);
    // Если требуется больше данных для публикации, добавьте здесь
}

}  // namespace ph_ec_sensor
}  // namespace esphome

