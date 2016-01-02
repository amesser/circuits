/*
 *  Copyright 2015 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of 3in1 Soil Moisture Sensor Calibrator firmware.
 *
 *  This software is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *  */
#ifndef CALIBRATOR_APP_HPP_
#define CALIBRATOR_APP_HPP_

#include "bsp.hpp"
#include "globals.hpp"
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
    TEMP_STATE_RETRYMEASUREMENT,
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
  uint16_t       m_LastHum;
  uint16_t       m_LastLight;

  uint8_t        m_LastTwiError;

  calibration_statistics m_CalStatistics;
  uint8_t                m_TwiBuffer[4];

  union {
    calibration_data      CalPrm;
  } m_Scratch;

  void        calculateCalibration();

  static void formatMinMax  (char * Buffer, struct CalibratorFit::calibration_minmax & cal, uint16_t value);
  static void formatLinRegr (char * Buffer, struct CalibratorFit::calibration_linearregression & cal, uint16_t value, uint16_t ref);
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
