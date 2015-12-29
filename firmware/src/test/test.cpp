/*
 *  Copyright 2015 Andreas Messer <andi@bastelmap.de>
 *
 *  This file is part of 3in1 Soil Moisture Sensor project.
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
#include "fit.hpp"

#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>

using namespace std;

class FitTest : public CppUnit::TestFixture
{
  CPPUNIT_TEST_SUITE(FitTest);
  CPPUNIT_TEST(testMinMax);
  CPPUNIT_TEST(testLinRegr);
  CPPUNIT_TEST_SUITE_END();

public:
  void testMinMax()
  {
    CalibratorFit::calibration_minmax stat;
    calibration_param                 prm;

    CalibratorFit::resetMinMaxStatistics(stat);

    CalibratorFit::collectMinMaxStatistics(stat, 100);
    CalibratorFit::collectMinMaxStatistics(stat, 200);

    CalibratorFit::calculateMinMaxCal(stat, prm);

    auto eval = static_cast<Evaluator*>(&prm);

    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(0),eval->scale(0));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(0),eval->scale(99));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(0),eval->scale(100));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(65535),eval->scale(200));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(65535),eval->scale(201));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(65535),eval->scale(65535));
  }

  void testLinRegr()
  {
    CalibratorFit::calibration_linearregression stat;
    calibration_param                           prm;

    CalibratorFit::resetLinRegrStat(stat);

    CalibratorFit::collectTempStatistics(stat, 1023-700, 273 - 10);
    CalibratorFit::collectTempStatistics(stat, 1023-600, 273 + 10);
    CalibratorFit::collectTempStatistics(stat, 1023-500, 273 + 29);

    CalibratorFit::calculateLinRegr(stat, prm);

    auto eval = static_cast<Evaluator*>(&prm);

    //CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(0), eval->scale(1023-701));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(273-10), eval->scale(1023-700));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(273+9), eval->scale(1023-600));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(273+29), eval->scale(1023-500));
    //CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(0xFFFF), eval->scale(1023-499));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(273+34), eval->scale(1023-475));
  }
};

CPPUNIT_TEST_SUITE_REGISTRATION(FitTest);

int main()
{
  CppUnit::TextUi::TestRunner runner;
  CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();

  runner.addTest( registry.makeTest() );
  bool wasSuccessful = runner.run("",false);
  return !wasSuccessful;
}
