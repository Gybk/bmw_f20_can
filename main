#include <Arduino.h>
#include <Arduino_CAN.h>
#include <ArduinoBLE.h>

// Объявление BLE сервиса и характеристик
BLEService racechronoService("00001ff8-0000-1000-8000-00805f9b34fb");
BLECharacteristic canBusMainCharacteristic("00000001-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, 20, false);
BLEUnsignedLongCharacteristic canBusFilterCharacteristic("00000002-0000-1000-8000-00805f9b34fb", BLEWrite);

// Буферы для отправки данных
uint8_t tempData[20] = {0};
uint8_t tempData2[20] = {0};

// Переменные для хранения данных
float batteryLevel;
int oilTemp;
int coolantTemp;
float rpm;
float accelPedal;
float throttle;
float steeringAngle;
int gear;
int brakeValue;
float wheelSpeedLeftRear;
float wheelSpeedRightRear;
float wheelSpeedLeftFront;
float wheelSpeedRightFront;
float engineTorque;
float gearboxTorque;
float inlineAccel;
float lateralAccel;
float yawRate;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Ready to receive data...");

  // Инициализация CAN
  if (!CAN.begin(CanBitRate::BR_500k)) { // Установите нужную скорость CAN-шины
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // Инициализация BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("RC DIY #0000");
  BLE.setAdvertisedService(racechronoService);
  racechronoService.addCharacteristic(canBusMainCharacteristic);
  racechronoService.addCharacteristic(canBusFilterCharacteristic);
  BLE.addService(racechronoService);

  // Инициализация первых 4 байт буферов
  ((uint32_t *)tempData)[0] = 0x0000ff01;
  ((uint32_t *)tempData2)[0] = 0x0000ff02;
  canBusMainCharacteristic.writeValue(tempData, 5);

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // Проверяем наличие подключения по BLE
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Чтение и обработка данных из CAN
      for (int i = 0; i < 30; i++) {
        checkCAN();
      }

      // Отправка данных по BLE
      sendViaBluetooth();
    }

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  } else {
    // Если нет подключения, продолжаем читать данные из CAN
    checkCAN();
  }
}

void checkCAN() {
  if (CAN.available()) {
    CanMsg const msg = CAN.read();

    switch (msg.id) {
      case 0x0A5: // RPM, Engine Torque, Gearbox Torque
        {
          // RPM
          rpm = ((msg.data[6] << 8) | msg.data[5]) * 0.25;
          Serial.print("RPM: ");
          Serial.println(rpm);

          // Engine Torque
          uint32_t rawTorque = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
          uint16_t torqueBits = (rawTorque >> 16) & 0x0FFF;
          engineTorque = (torqueBits * 0.5) - 1023.5;
          Serial.print("Engine Torque: ");
          Serial.println(engineTorque);

          // Gearbox Torque
          torqueBits = (rawTorque >> 28) & 0x0FFF;
          gearboxTorque = (torqueBits * 0.5) - 1023.5;
          Serial.print("Gearbox Torque: ");
          Serial.println(gearboxTorque);

          // Упаковка данных для отправки по BLE
          uint16_t rpmInt = (uint16_t)(rpm);
          tempData[4] = (rpmInt >> 8) & 0xFF;
          tempData[5] = rpmInt & 0xFF;

          int16_t engineTorqueInt = (int16_t)(engineTorque * 10);
          tempData2[9] = (engineTorqueInt >> 8) & 0xFF;
          tempData2[10] = engineTorqueInt & 0xFF;

          int16_t gearboxTorqueInt = (int16_t)(gearboxTorque * 10);
          tempData2[11] = (gearboxTorqueInt >> 8) & 0xFF;
          tempData2[12] = gearboxTorqueInt & 0xFF;
        }
        break;

      case 0x0D9: // Accelerator Pedal, Throttle
        {
          // Accelerator Pedal
          uint16_t rawAccel = (msg.data[3] << 8) | msg.data[2];
          uint16_t accelBits = rawAccel & 0x0FFF;
          accelPedal = accelBits * 0.025;
          Serial.print("Accelerator Pedal: ");
          Serial.print(accelPedal);
          Serial.println("%");

          // Throttle
          uint16_t rawThrottle = (msg.data[5] << 8) | msg.data[4];
          uint16_t throttleBits = rawThrottle & 0x0FFF;
          throttle = throttleBits * 0.025;
          Serial.print("Throttle: ");
          Serial.println(throttle);

          // Упаковка данных для отправки по BLE
          uint8_t throttleInt = (uint8_t)(throttle * 2); // Масштабирование до 0-200
          tempData[6] = throttleInt;
        }
        break;

      case 0x0EF: // Brake
        {
          int brakeRaw = msg.data[3];
          brakeValue = (brakeRaw * -2) + 250;
          Serial.print("Brake: ");
          Serial.println(brakeValue);

          // Упаковка данных для отправки по BLE
          tempData[18] = (uint8_t)(brakeValue);
        }
        break;

      case 0x302: // Steering Angle
        {
          int rawValue = (msg.data[3] << 8) | msg.data[2];  // Little Endian
          steeringAngle = rawValue * 0.04395 - 1440.11;
          Serial.print("Steering Angle: ");
          Serial.println(steeringAngle);

          // Упаковка данных для отправки по BLE
          int16_t steeringAngleInt = (int16_t)(steeringAngle);
          tempData[8] = (steeringAngleInt >> 8) & 0xFF;
          tempData[9] = steeringAngleInt & 0xFF;
        }
        break;

      case 0x254: // Wheel Speed
        {
          // Wheel Speed - Left Rear (Offset: 0)
          wheelSpeedLeftRear = ((msg.data[1] << 8) | msg.data[0]) * 0.015625 - 511.984;
          Serial.print("Wheel Speed - Left Rear: ");
          Serial.println(wheelSpeedLeftRear);

          // Wheel Speed - Right Rear (Offset: 2)
          wheelSpeedRightRear = ((msg.data[3] << 8) | msg.data[2]) * 0.015625 - 511.984;
          Serial.print("Wheel Speed - Right Rear: ");
          Serial.println(wheelSpeedRightRear);

          // Wheel Speed - Left Front (Offset: 4)
          wheelSpeedLeftFront = ((msg.data[5] << 8) | msg.data[4]) * 0.015625 - 511.984;
          Serial.print("Wheel Speed - Left Front: ");
          Serial.println(wheelSpeedLeftFront);

          // Wheel Speed - Right Front (Offset: 6)
          wheelSpeedRightFront = ((msg.data[7] << 8) | msg.data[6]) * 0.015625 - 511.984;
          Serial.print("Wheel Speed - Right Front: ");
          Serial.println(wheelSpeedRightFront);

          // Упаковка данных для отправки по BLE
          uint16_t wheelSpeedLRInt = (uint16_t)((wheelSpeedLeftRear + 10000) * 100);
          tempData[10] = (wheelSpeedLRInt >> 8) & 0xFF;
          tempData[11] = wheelSpeedLRInt & 0xFF;

          uint16_t wheelSpeedRRInt = (uint16_t)((wheelSpeedRightRear + 10000) * 100);
          tempData[12] = (wheelSpeedRRInt >> 8) & 0xFF;
          tempData[13] = wheelSpeedRRInt & 0xFF;

          uint16_t wheelSpeedLFInt = (uint16_t)((wheelSpeedLeftFront + 10000) * 100);
          tempData[14] = (wheelSpeedLFInt >> 8) & 0xFF;
          tempData[15] = wheelSpeedLFInt & 0xFF;

          uint16_t wheelSpeedRFInt = (uint16_t)((wheelSpeedRightFront + 10000) * 100);
          tempData[16] = (wheelSpeedRFInt >> 8) & 0xFF;
          tempData[17] = wheelSpeedRFInt & 0xFF;
        }
        break;

      case 0x281: // Battery Level
        {
          batteryLevel = (((msg.data[0] << 8) | msg.data[1]) & 0xFFF) * 15.0 / 4096.0;
          Serial.print("Battery Level: ");
          Serial.println(batteryLevel);

          // Упаковка данных для отправки по BLE
          uint16_t batteryLevelInt = (uint16_t)(batteryLevel * 1000);
          tempData2[4] = (batteryLevelInt >> 8) & 0xFF;
          tempData2[5] = batteryLevelInt & 0xFF;
        }
        break;

      case 0x3F9:
        {
          oilTemp = msg.data[5] - 48;
          coolantTemp = msg.data[4] - 48;
          gear = (msg.data[6] & 0xF) - 4;
          Serial.print("Oil Temp: ");
          Serial.println(oilTemp);
          Serial.print("Coolant Temp: ");
          Serial.println(coolantTemp);
          Serial.print("Gear: ");
          Serial.println(gear);

          // Упаковка данных для отправки по BLE
          tempData2[6] = (uint8_t)(oilTemp);
          tempData2[7] = (uint8_t)(coolantTemp);
          tempData2[8] = (int8_t)(gear);
        }
        break;

      case 0x199: // Inline Acceleration
        {
          uint16_t rawValue = (msg.data[3] << 8) | msg.data[2];
          inlineAccel = (rawValue * 0.002) - 65.0;
          Serial.print("Inline Accel: ");
          Serial.println(inlineAccel);

          // Упаковка данных для отправки по BLE
          int16_t inlineAccelInt = (int16_t)(inlineAccel * 1000);
          tempData2[13] = (inlineAccelInt >> 8) & 0xFF;
          tempData2[14] = inlineAccelInt & 0xFF;
        }
        break;

      case 0x19A: // Lateral Acceleration
        {
          uint16_t rawValue = (msg.data[3] << 8) | msg.data[2];
          lateralAccel = (rawValue * 0.002) - 65.0;
          Serial.print("Lateral Accel: ");
          Serial.println(lateralAccel);

          // Упаковка данных для отправки по BLE
          int16_t lateralAccelInt = (int16_t)(lateralAccel * 1000);
          tempData2[15] = (lateralAccelInt >> 8) & 0xFF;
          tempData2[16] = lateralAccelInt & 0xFF;
        }
        break;

      case 0x19F: // Yaw Rate (Gyro)
        {
          uint16_t rawValue = (msg.data[3] << 8) | msg.data[2];
          yawRate = (rawValue * 0.005) - 163.84;
          Serial.print("Yaw Rate: ");
          Serial.println(yawRate);

          // Упаковка данных для отправки по BLE
          int16_t yawRateInt = (int16_t)(yawRate * 1000);
          tempData2[17] = (yawRateInt >> 8) & 0xFF;
          tempData2[18] = yawRateInt & 0xFF;
        }
        break;

      // Добавьте другие сообщения при необходимости

      default:
        // Необработанный ID сообщения
        // Serial.print("Unhandled ID: 0x");
        // Serial.println(msg.id, HEX);
        break;
    }
  }
}

void sendViaBluetooth() {
  // Отправка первого сообщения
  ((uint32_t *)tempData)[0] = 0x0000ff01;
  canBusMainCharacteristic.writeValue(tempData, 20, false);

  // Небольшая задержка
  delay(10);

  // Отправка второго сообщения
  ((uint32_t *)tempData2)[0] = 0x0000ff02;
  canBusMainCharacteristic.writeValue(tempData2, 20, false);

  // Небольшая задержка
  delay(10);
}
