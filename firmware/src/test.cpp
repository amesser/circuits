/*
 * test.cpp
 *
 *  Created on: 17.10.2015
 *      Author: andi
 */
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

    CalibratorFit::collectMinMaxStatistics(stat, 100, true);
    CalibratorFit::collectMinMaxStatistics(stat, 200, true);

    CalibratorFit::calculateMinMaxCal(stat, prm);

    auto eval = static_cast<Evaluator*>(&prm);

    CPPUNIT_ASSERT(eval->scale(0)  == 0);
    CPPUNIT_ASSERT(eval->scale(99) == 0);
    CPPUNIT_ASSERT(eval->scale(100) == 0);
    CPPUNIT_ASSERT(eval->scale(200) == 65535);
    CPPUNIT_ASSERT(eval->scale(201) == 65535);
    CPPUNIT_ASSERT(eval->scale(65535) == 65535);
  }

  void testLinRegr()
  {
    CalibratorFit::calibration_linearregression stat;
    calibration_param                           prm;

    CalibratorFit::resetLinRegrStat(stat);

    CalibratorFit::collectTempStatistics(stat, 0-700, 273 - 10);
    CalibratorFit::collectTempStatistics(stat, 0-500, 273 + 29);

    CalibratorFit::calculateLinRegr(stat, prm);

    auto eval = static_cast<Evaluator*>(&prm);

    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(0), eval->scale(0-701));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(273-10), eval->scale(0-700));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(273+10), eval->scale(0-600));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(273+29), eval->scale(0-500));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(0xFFFF), eval->scale(0-499));
    CPPUNIT_ASSERT_EQUAL(static_cast<uint16_t>(273+35), eval->scale(0-475));
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
