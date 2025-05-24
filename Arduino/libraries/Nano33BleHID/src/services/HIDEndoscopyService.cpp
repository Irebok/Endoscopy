#include <mbed.h>
#include "services/HIDEndoscopyService.h"

namespace {

// Report Reference
static report_reference_t input_report_ref = { 0, INPUT_REPORT };

static GattAttribute input_report_ref_desc(
  ATT_UUID_HID_REPORT_ID_MAPPING,
  (uint8_t*)&input_report_ref,
  sizeof(input_report_ref),
  sizeof(input_report_ref)
);

static GattAttribute *input_report_ref_descs[] = {
  &input_report_ref_desc,
};

// Input Report
#pragma pack(push, 1)
struct {
  int16_t axisAX;
  int16_t axisAY;
  int16_t axisAZ;
  int16_t axisGX;
  int16_t axisGY;
  int16_t axisGZ;
  int16_t axisMX;
  int16_t axisMY;
  int16_t axisMZ;
  uint8_t id;
  uint8_t enable;  
} hid_input_report;
#pragma pack(pop)


// #pragma pack(push, 1)
// struct {
//   float axisX;
//   float axisY;
//   float axisZ;
//   uint32_t dt;
//   uint8_t id;
//   uint8_t enable;  
//   uint8_t timestamp;  
//   uint8_t buttons;
// } hid_input_report;
// #pragma pack(pop)

// Report Map
static uint8_t hid_report_map[] =
{
  USAGE_PAGE(1),      0x01,       // Usage Page (Generic Desktop)
  USAGE(1),           0x05,       // Usage (Game Pad)
  COLLECTION(1),      0x01,       // Collection (Application)
    USAGE(1),           0x01,       // Usage (Pointer)
    COLLECTION(1),      0x00,       // Collection (Physical)

    // Array 1
      USAGE_PAGE(1),      0x01,       // Usage Page (Generic Desktop)
      USAGE(1),           0x30,       // Usage (acelX)
      USAGE(1),           0x31,       // Usage (acelY)
      USAGE(1),           0x32,       // Usage (acelZ)
      REPORT_SIZE(1),     0x10,       // Report Size (16)
      REPORT_COUNT(1),    0x09,       // Report Count (3)
      INPUT(1),           0x02,       // Input (Data, Variable, Absolute)

      USAGE_PAGE(1),      0x01,       // Usage Page (Generic Desktop)
      USAGE(1),           0x34,       // Usage (id)
      REPORT_SIZE(1),     0x08,       // Report Size (8)
      REPORT_COUNT(1),    0x01,       // Report Count (1)
      INPUT(1),           0x02,       // Input (Data, Variable, Absolute)

      USAGE_PAGE(1),      0x01,       // Usage Page (Generic Desktop)
      USAGE(1),           0x35,       // Usage (enable)
      REPORT_SIZE(1),     0x08,       // Report Size (8)
      REPORT_COUNT(1),    0x01,       // Report Count (1)
      INPUT(1),           0x02,       // Input (Data, Variable, Absolute)
      
    END_COLLECTION(0),              // End Collection (Physical)
  END_COLLECTION(0),              // End Collection (Application)
};

} // namespace "" 


HIDEndoscopyService::HIDEndoscopyService(BLE &_ble) 
  : HIDService(_ble,
               HID_OTHER,
               
               // report map
               hid_report_map, 
               sizeof(hid_report_map) / sizeof(*hid_report_map),

               // input report
               (uint8_t*)&hid_input_report,
               sizeof(hid_input_report),
               input_report_ref_descs,
               sizeof(input_report_ref_descs) / sizeof(*input_report_ref_descs))
{

}

void HIDEndoscopyService::motion(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz, uint8_t id, uint8_t data_enable) {
  hid_input_report.axisAX = (int16_t)(ax);
  hid_input_report.axisAY = (int16_t)(ay);
  hid_input_report.axisAZ = (int16_t)(az);
  hid_input_report.axisGX = (int16_t)(gx);
  hid_input_report.axisGY = (int16_t)(gy);
  hid_input_report.axisGZ = (int16_t)(gz);
  hid_input_report.axisMX = (int16_t)(mx);
  hid_input_report.axisMY = (int16_t)(my);
  hid_input_report.axisMZ = (int16_t)(mz);
  hid_input_report.id = id;
  hid_input_report.enable = data_enable;
}

// void HIDEndoscopyService::motionAcel(float fx, float fy, float fz, uint8_t enable, uint32_t dt, uint8_t timestamp) {
//   hid_input_report.axisX = fx;
//   hid_input_report.axisY = fy;
//   hid_input_report.axisZ = fz;
//   hid_input_report.enable = enable & 0b00000001;
//   hid_input_report.dt = dt;
//   hid_input_report.id = 1;
//   hid_input_report.timestamp = timestamp;
// }

// void HIDEndoscopyService::motionGyro(float fx, float fy, float fz, uint8_t enable, uint32_t dt, uint8_t timestamp) {
//   hid_input_report.axisX = fx;
//   hid_input_report.axisY = fy;
//   hid_input_report.axisZ = fz;
//   hid_input_report.enable = enable & 0b00000010;
//   hid_input_report.dt = dt;
//   hid_input_report.id = 2;
//   hid_input_report.timestamp = timestamp;
// }

// void HIDEndoscopyService::motionMag(float fx, float fy, float fz, uint8_t enable, uint32_t dt, uint8_t timestamp) {
//   hid_input_report.axisX = fx;
//   hid_input_report.axisY = fy;
//   hid_input_report.axisZ = fz;
//   hid_input_report.enable = enable & 0b00000100;
//   hid_input_report.dt = dt;
//   hid_input_report.id = 3;
//   hid_input_report.timestamp = timestamp;
// }


// void HIDEndoscopyService::buttons(uint8_t buttons) {
//   hid_input_report.buttons = uint8_t(buttons); 
// }


