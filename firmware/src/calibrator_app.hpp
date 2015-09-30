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
    TEMP_STATE_READMEASUREMENT,
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

  char           m_LcdScratch[16];

  uint8_t        m_LastTwiError;

  calibration_statistics m_CalStatistics;

  union {
    calibration_data      CalPrm;
  } m_Scratch;

  static void collectMinMaxStatistics(struct calibration_minmax &stat, uint_fast16_t Counts, bool ForceCollect = false);
  static void collectTempStatistics(struct calibration_linearregression &stat, uint_fast16_t TempCounts, uint_fast16_t Temp);

  void        collectStatistics();
  void        calculateCalibration();
  static void calculateMinMaxCal(struct calibration_minmax &stat, struct calibration_param &prm);
  static void calculateLinRegr(struct calibration_linearregression &stat, struct calibration_param &prm);

  void formatMinMax (struct calibration_minmax & cal, uint16_t value);
  void formatLinRegr   (struct calibration_linearregression & cal, uint16_t value, uint16_t ref);
public:
  void changeState(    enum AppState  NextState);

  void handleUiState();
  void changeUiState(enum UiState NextState);

  void changeTempState(enum TempState NextState);
  void handleTempState();

  uint16_t getValue(uint_fast8_t idx, uint_fast16_t input);
  void     displayValue(uint_fast8_t idx);


  void readSensor();
  void handleKeys();
  void handleState();
};


#endif /* FIRMWARE_SRC_CALIBRATOR_CALIBRATOR_APP_HPP_ */
