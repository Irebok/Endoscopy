#include "Nano33BleHID.h"
#include "signal_utils.h"
#include <Arduino_LSM9DS1.h>
#include <Arduino_LSM9DS1Extern.h>
#include <Wire.h>
#include <Encoder.h>


/* -------------------------------------------------------------------------- */

Nano33BleEndoscopy bleEndoscopy("Endoscopy device");
// LSM9DS1 imuEndoscopyPunta;
// LSM9DS1 imuEndoscopyTubo;
LSM9DS1ExternClass imuEndoscopyPunta(Wire, 0x6B, 0x1E);
LSM9DS1ExternClass imuEndoscopyTubo(Wire, 0x6A, 0x1C);

Encoder encoder(2, 3);


// // // Builtin LED intensity when connected.
static const int kLedConnectedIntensity = 50;

/* -------------------------------------------------------------------------- */
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
unsigned long dt_a=0, dt_a_new=0, dt_g=0, dt_g_new=0, dt_m=0, dt_m_new=0;
int degreesX = 0;
int degreesY = 0;
int degreesZ = 0;

int mode = 0;
const int max_modes = 2;
const int threshold = 2;
const int modepin = A7;

int count = 0;
float id = 1;
uint8_t timestamp = 0;
uint8_t data_enable1 = 0b00000000;
uint8_t data_enable2 = 0b00000000;
uint8_t data_enable3 = 0b00000000;


static uint32_t sendCount = 0;
static uint32_t startTime = 0;
static bool firstEntry = true;
int avoid_mag = 0;
int select_sensor = 0;
const int SAMPLE_INTERVAL_MS = 5;
static unsigned long last_sample_time = 0;
long oldPositionEncoder  = -999;
uint16_t bits_mayores=0;
uint16_t bits_menores=0;


const int numSamples = 100;

float samples[numSamples];
int sampleIndex = 0;
bool filled = false;
/* -------------------------------------------------------------------------- */


void enviar_sensor_interno(auto *kb)
{
      data_enable1 = 0b00000000;
      if (IMU.accelerationAvailable()) {
        IMU.readAccelerationInteger(ax, ay, az);
        data_enable1 |= (1 << 0);
      }
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscopeInteger(gx, gy, gz);
        data_enable1 |= (1 << 1);
      }
      // if(avoid_mag >=3)
      // {
        if (IMU.magneticFieldAvailable()) {
            IMU.readMagneticFieldInteger(mx, my, mz);
            data_enable1 |= (1 << 2);
        }
      // }
      // Serial.print("[INTERNO ] Acc: ");
      // Serial.print(ax); Serial.print(", ");
      // Serial.print(ay); Serial.print(", ");
      // Serial.print(az); Serial.print(" | Gyro: ");
      // Serial.print(gx); Serial.print(", ");
      // Serial.print(gy); Serial.print(", ");
      // Serial.print(gz); Serial.print(" | Mag: ");
      // Serial.print(mx); Serial.print(", ");
      // Serial.print(my); Serial.print(", ");
      // Serial.println(mz);
      kb->motion(ax, ay, az, gx, gy, gz, mx, my, mz, 1, data_enable1);
        kb->SendReport();
}


void enviar_sensor_tubo(auto *kb)
{
      data_enable2 = 0b00000000;
      if (imuEndoscopyTubo.accelerationAvailable()) {
        imuEndoscopyTubo.readAccelerationInteger(ax, ay, az);
        data_enable2 |= (1 << 0);
      }
      if (imuEndoscopyTubo.gyroscopeAvailable()) {
        imuEndoscopyTubo.readGyroscopeInteger(gx, gy, gz);
        data_enable2 |= (1 << 1);
      }
      // if(avoid_mag >=3)
      // {
        if (imuEndoscopyTubo.magneticFieldAvailable()) {
            imuEndoscopyTubo.readMagneticFieldInteger(mx, my, mz);
            data_enable2 |= (1 << 2);
        }
      // }
      // Serial.print("[TUBO ] Acc: ");
      // Serial.print(ax); Serial.print(", ");
      // Serial.print(ay); Serial.print(", ");
      // Serial.print(az); Serial.print(" | Gyro: ");
      // Serial.print(gx); Serial.print(", ");
      // Serial.print(gy); Serial.print(", ");
      // Serial.print(gz); Serial.print(" | Mag: ");
      // Serial.print(mx); Serial.print(", ");
      // Serial.print(my); Serial.print(", ");
      // Serial.println(mz);
      kb->motion(ax, ay, az, gx, gy, gz, mx, my, mz, 2, data_enable2);
        kb->SendReport();
}

void enviar_sensor_punta(auto *kb)
{
      data_enable3 = 0b00000000;
      if (imuEndoscopyPunta.accelerationAvailable()) {
        imuEndoscopyPunta.readAccelerationInteger(ax, ay, az);
        data_enable3 |= (1 << 0);
      }
      if (imuEndoscopyPunta.gyroscopeAvailable()) {
        imuEndoscopyPunta.readGyroscopeInteger(gx, gy, gz);
        data_enable3 |= (1 << 1);
      }
      // if(avoid_mag >=3)
      // {
        if (imuEndoscopyPunta.magneticFieldAvailable()) {
            imuEndoscopyPunta.readMagneticFieldInteger(mx, my, mz);
            data_enable3 |= (1 << 2);
        }
      // }
      // Serial.print("[PUNTA ] Acc: ");
      // Serial.print(ax); Serial.print(", ");
      // Serial.print(ay); Serial.print(", ");
      // Serial.print(az); Serial.print(" | Gyro: ");
      // Serial.print(gx); Serial.print(", ");
      // Serial.print(gy); Serial.print(", ");
      // Serial.print(gz); Serial.print(" | Mag: ");
      // Serial.print(mx); Serial.print(", ");
      // Serial.print(my); Serial.print(", ");
      // Serial.println(mz);
      kb->motion(ax, ay, az, gx, gy, gz, mx, my, mz, 3, data_enable3);
        kb->SendReport();
}

void enviar_encoder(auto *kb)
{  
  long newPositionEncoder = encoder.read();
  if (newPositionEncoder != oldPositionEncoder) {
    oldPositionEncoder = newPositionEncoder;
    Serial.println(newPositionEncoder);
    bits_mayores = (uint16_t)(newPositionEncoder >> 16);
    bits_menores = (uint16_t)(newPositionEncoder & 0xFFFF);
  }
  kb->motion(bits_mayores, bits_menores, 0, 0, 0, 0, 0, 0, 0, 4, 1);
    kb->SendReport();
}

void enviar_buttons(auto *kb)
{  
  int8_t button_enable = 0b00000000;
  float sensorValue = analogRead(A7);
  if(sensorValue<240)
  {
    button_enable = 0b00000000;
  }
  else if(sensorValue<270)
  {
    button_enable = 0b00000010;
  }
  else if(sensorValue<300)
  {
    button_enable = 0b00000100;
  }
  else if(sensorValue>300)
  {
    button_enable = 0b00000110;
  }
  kb->motion(0, 0, 0, 0, 0, 0, 0, 0, 0, 5, button_enable);
    kb->SendReport();
  // Serial.print("Button: ");
  //   Serial.println(button_enable);
}

void setup()
{
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Setup started...");

  // General setup.
  pinMode(LED_BUILTIN, OUTPUT);

  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  Serial.print("Magnetometer sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");

  Wire.begin();

  imuEndoscopyPunta.begin();
  imuEndoscopyTubo.begin();

  bleEndoscopy.initialize();

  Serial.println("Setup finished...");

  MbedBleHID_RunEventThread();
}

void loop()
{
      if (bleEndoscopy.connected() == false) {
        Serial.println("Conexion cerrada");
        analogWrite(LED_BUILTIN, 0);
        return;
      }
      
      auto *kb = bleEndoscopy.hid();
      avoid_mag++;
      sendCount ++;
      switch (sendCount) {
        case 1: enviar_sensor_interno(kb); break;
        case 2: enviar_sensor_tubo(kb); break;
        case 3: enviar_sensor_punta(kb); break;
        case 4: enviar_encoder(kb); break;
        case 5: enviar_buttons(kb); break;
      }
      if(sendCount>=5) sendCount=0;
}