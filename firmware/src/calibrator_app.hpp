/*
 * calibrator_app.hpp
 *
 *  Created on: 10.08.2015
 *      Author: andi
 */

#ifndef CALIBRATOR_APP_HPP_
#define CALIBRATOR_APP_HPP_

using namespace ecpp;

#include "calibrator/bsp.hpp"
#include "calibrator/globals.hpp"

using namespace ecpp;

struct calibration_minmax
{
  uint16_t Max;
  uint16_t Min;
};

struct calibration_linearregression
{
  uint8_t NumPoints;
  int32_t SumX;
  int32_t SumY;
};

struct calibration_statistics
{
  struct calibration_minmax           Humidity;
  struct calibration_minmax           Light;
  struct calibration_linearregression Temperature;
};
class CalibratorApp
{
public:
  enum AppState
  {
    APP_STATE_POWERUP = 0,
    APP_STATE_IDLE,
    APP_STATE_READSENSOR,
    APP_STATE_WRITESENSOR,
    APP_STATE_MAX,
  };

  enum MenuState
  {
    MENU_STATE_READSENSOR = 0,
    MENU_STATE_HUMIDITY,
    MENU_STATE_LIGHT,
    MENU_STATE_TEMP,
    MENU_STATE_RESETSENSOR,
    MENU_STATE_IDLE,
  };

  /* define a type for member function pointers without an argument*/
  typedef void (CalibratorApp::* Handler)();

private:
  uint_fast8_t   m_State;
  uint_fast8_t   m_MenuState;
  uint_fast8_t   m_KeyState;

  calibration_statistics m_CalStatistics;

  union {
    calibration_data      CalPrm;
  } m_Scratch;

  template<typename T>
  void invoke(const T& ptr);

  void invoke(Handler handler);

  void        evaluateSensordata();
  void        calculateCalibration();
  static void calculateMinMaxCal(struct calibration_minmax &stat, struct calibration_param &prm);
  static void calculateLinRegr(struct calibration_linearregression &stat, struct calibration_param &prm);
public:
  void leaveStatePowerUp();

  void enterStateIdle();
  void pollStateIdle();

  void leaveStateReadSensor();
  void enterStateReadSensor();
  void pollStateReadSensor();

  void enterStateWriteSensor();
  void pollStateWriteSensor();

  void changeState(    enum AppState  NextState);
  void changeMenuState(enum MenuState NextState);

  uint16_t getValue(uint_fast8_t idx, uint_fast16_t input);
  void     displayValue(uint_fast8_t idx);
  void     displayHumidity();
  void     displayLight();
  void     displayTemperature();

  void readSensor();
  void handleKeys();
  void handleState();
};

template<typename T>
void CalibratorApp::invoke(const T & ptr)
{
  Handler handler = ptr;
  invoke(handler);
}

void CalibratorApp::invoke(Handler handler)
{
  if(handler != NULL)
  {
    (this->*handler)();
  }
}

#endif /* FIRMWARE_SRC_CALIBRATOR_CALIBRATOR_APP_HPP_ */
