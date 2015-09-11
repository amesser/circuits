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

/** Hold the data used for linear regression
 *
 * In order to avoid to strong influence of lots
 * of calibration points in one range, the x/y measurement
 * pair will be grouped in slots. each slot is 2 degrees
 * wide. When calculating the calibration factors, each slot
 * will be weighted with the inverse of its contents
 */
struct calibration_linearregression
{
  int16_t  SumX[25];
  int16_t  SumY[25];
  int32_t  SumXY[25];
  int32_t  SumXX[25];
  uint8_t NumPoints[25];
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
    MENU_STATE_RESET,
    MENU_STATE_IDLE,
  };

  /* define a type for member function pointers with a single argument*/
  typedef void (CalibratorApp::* Handler)(uint_fast16_t Argument);

private:
  uint_fast8_t   m_State;
  uint_fast8_t   m_MenuState;
  uint_fast8_t   m_KeyState;

  /** bitmask defining which value of the current readout has not yet been collected */
  uint_fast8_t   m_CalStatMask;

  calibration_statistics m_CalStatistics;

  union {
    calibration_data      CalPrm;
  } m_Scratch;

  template<typename T>
  void invokeFromList(const T & HandlerList, uint_fast16_t Index, uint_fast16_t Argument);

  static void collectMinMaxStatistics(struct calibration_minmax &stat, uint_fast16_t Counts, bool ForceCollect = false);
  static void collectTempStatistics(struct calibration_linearregression &stat, uint_fast16_t TempCounts, uint_fast16_t Temp);

  void        collectStatistics();
  void        calculateCalibration();
  static void calculateMinMaxCal(struct calibration_minmax &stat, struct calibration_param &prm);
  static void calculateLinRegr(struct calibration_linearregression &stat, struct calibration_param &prm);
public:
  void leaveStatePowerUp(uint_fast16_t Argument);

  void enterStateIdle(uint_fast16_t Argument);
  void pollStateIdle(uint_fast16_t Argument);

  void leaveStateReadSensor(uint_fast16_t Argument);
  void enterStateReadSensor(uint_fast16_t Argument);
  void pollStateReadSensor(uint_fast16_t Argument);

  void enterStateWriteSensor(uint_fast16_t Argument);
  void pollStateWriteSensor(uint_fast16_t Argument);

  void changeState(    enum AppState  NextState);
  void changeMenuState(enum MenuState NextState);

  uint16_t getValue(uint_fast8_t idx, uint_fast16_t input);
  void     displayValue(uint_fast8_t idx);

  void     handleMenuReadSensor(uint_fast16_t Argument);
  void     handleMenuHumidity(uint_fast16_t Argument);
  void     handleMenuLight(uint_fast16_t Argument);
  void     handleMenuTemperature(uint_fast16_t Argument);
  void     handleMenuEraseSensor(uint_fast16_t Argument);
  void     handleMenuReset(uint_fast16_t Argument);

  void readSensor();
  void handleKeys();
  void handleState();
};


template<typename T>
void CalibratorApp::invokeFromList(const T & HandlerList, uint_fast16_t Index, uint_fast16_t Argument)
{
  Handler h = HandlerList[Index];

  if(0 != h)
  {
    (this->*h)(Argument);
  }
}


#endif /* FIRMWARE_SRC_CALIBRATOR_CALIBRATOR_APP_HPP_ */
