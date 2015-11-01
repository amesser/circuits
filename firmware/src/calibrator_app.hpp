/*
 * calibrator_app.hpp
 *
 *  Created on: 10.08.2015
 *      Author: andi
 */

#ifndef CALIBRATOR_APP_HPP_
#define CALIBRATOR_APP_HPP_

#include "calibrator/bsp.hpp"
#include "calibrator/globals.hpp"
#include "fit.hpp"

using namespace ecpp;

struct calibration_statistics
{
  struct CalibratorFit::calibration_minmax           Humidity;
  struct CalibratorFit::calibration_minmax           Light;
  struct CalibratorFit::calibration_linearregression Temperature;
};
class CalibratorApp
{
private:
  typedef CalibratorBsp::RowBufferType LcdRowBufferType;
public:
  enum AppState
  {
    APP_STATE_POWERUP = 0,
    APP_STATE_IDLE,
    APP_STATE_READSENSOR,
    APP_STATE_WRITESENSOR,
  };

  /* state of the user interface */
  enum UiState
  {
    UI_STATE_READSENSOR = 0,
    UI_STATE_READINGSENSOR,
    UI_STATE_READSENSORRESULTS,
    UI_STATE_READSENSORRAW,
    UI_STATE_READSENSORHUMIDITY,
    UI_STATE_READSENSORLIGHT,
    UI_STATE_READSENSORTEMPERATURE,
    UI_STATE_SENSORERROR,
    UI_STATE_ERASESENSOR,
    UI_STATE_ERASINGSENSOR,
    UI_STATE_CALIBRATESENSOR,
    UI_STATE_CALIBRATINGSENSOR,
    UI_STATE_RESETSTATISTICS,
    UI_STATE_AUTOCALIBRATION,
    UI_STATE_AUTOSELECTINTERVAL,
    UI_STATE_AUTOREADINGSENSOR,
    UI_STATE_AUTOWAIT,
    UI_STATE_STARTUP,
  };

  /** State of temperatur handler */
  enum TempState
  {
    TEMP_STATE_POWERON = 0,
    TEMP_STATE_WSETUP,
    TEMP_STATE_WTRIGGER,
    TEMP_STATE_WMEASUREMENT,
    // TEMP_STATE_READMEASUREMENT,
    TEMP_STATE_WTIMEOUT,
  };

private:
  uint_fast8_t   m_State;
  uint_fast8_t   m_MenuState;
  uint_fast8_t   m_KeyState;
  uint_fast8_t   m_TempState;
  uint_fast8_t   m_AutoIntervalSelector;

  uint8_t        m_StableCnt;
  uint8_t        m_Rh;
  uint16_t       m_AbsTemp;

  uint8_t        m_LastTwiError;

  calibration_statistics m_CalStatistics;

  union {
    calibration_data      CalPrm;
  } m_Scratch;

  void        calculateCalibration();

  static void formatMinMax  (LcdRowBufferType & Buffer, struct CalibratorFit::calibration_minmax & cal, uint16_t value);
  static void formatLinRegr (LcdRowBufferType & Buffer, struct CalibratorFit::calibration_linearregression & cal, uint16_t value, uint16_t ref);
public:
  void changeState(    enum AppState  NextState);

  void handleUiState();
  void changeUiState(enum UiState NextState);

  void changeTempState(enum TempState NextState);
  void handleTempState();

  uint16_t getValue(uint_fast8_t idx, uint_fast16_t input);

  void readSensor();
  void handleKeys();
  void handleState();
};


#endif /* FIRMWARE_SRC_CALIBRATOR_CALIBRATOR_APP_HPP_ */
