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

class CalibratorApp
{
public:

private:
  enum {
    STATE_POWERUP = 0,
    STATE_READSENSOR,
    STATE_READDONE
  };

  uint8_t              m_State;

public:
  void readSensor();
  void handleState();
};



#endif /* FIRMWARE_SRC_CALIBRATOR_CALIBRATOR_APP_HPP_ */
