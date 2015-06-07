/*
 * controller_ui.hpp
 *
 *  Created on: 05.06.2015
 *      Author: andi
 */

#ifndef FIRMWARE_SRC_CONTROLLER_UI_HPP_
#define FIRMWARE_SRC_CONTROLLER_UI_HPP_

#include "ecpp/Target.hpp"

class UserInterface
{
private:
  uint8_t m_Keymask;
  uint8_t m_CurrentMenuItem;
  uint8_t m_Edit;

  void updateStatusLine();
public:
  void eventSecondTick() {updateStatusLine();}

  bool isEditing() const {return m_Edit != 0;}
  void cycle();
};

extern UserInterface ui;

#endif /* FIRMWARE_SRC_CONTROLLER_UI_HPP_ */
