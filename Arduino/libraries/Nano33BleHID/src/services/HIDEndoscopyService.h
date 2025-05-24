#ifndef BLE_HID_ENDOSCOPY_SERVICE_H__
#define BLE_HID_ENDOSCOPY_SERVICE_H__

#if BLE_FEATURE_GATT_SERVER

#include "services/HIDService.h"

/* -------------------------------------------------------------------------- */

/**
 * BLE HID Game Pad Service
 *
 * @par usage
 *
 * When this class is instantiated, it adds a Game Pad HID service in 
 * the GattServer.
 *
 * The ENDOSCOPY consist of a joystick for X and Y motion and 4 buttons.
 *
 * @attention Multiple instances of this hid service are not supported.
 * @see HIDService
 */
class HIDEndoscopyService : public HIDService {
 public:
  enum Button {
    BUTTON_NONE    = 0,
    BUTTON_A       = 1 << 0,
    BUTTON_B       = 1 << 1,
    BUTTON_X       = 1 << 2,
    BUTTON_Y       = 1 << 3,
  };

  HIDEndoscopyService(BLE &_ble);

  ble::adv_data_appearance_t appearance() const override {
    return ble::adv_data_appearance_t::GAMEPAD;
  }
  // void motion();
  void motion(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz, uint8_t id, uint8_t data_enable);

  // void motionAcel(float fx, float fy, float fz, uint8_t enable, uint32_t dt, uint8_t timestamp);
  // void motionGyro(float fx, float fy, float fz, uint8_t enabl, uint32_t dt, uint8_t timestamp);
  // void motionMag(float fx, float fy, float fz, uint8_t enable, uint32_t dt, uint8_t timestamp);
  // void avaliableData(uint8_t enable_data);
  // void button(Button buttons);
  void buttons(uint8_t buttons); 
};

/* -------------------------------------------------------------------------- */

#endif // BLE_FEATURE_GATT_SERVER

#endif // BLE_HID_ENDOSCOPY_SERVICE_H__