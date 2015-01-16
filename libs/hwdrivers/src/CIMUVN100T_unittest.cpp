/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/config.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/hwdrivers/CIMUVN100T.h>
#include <mrpt/obs/CObservationIMU.h>

#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;

TEST(CIMUVN100T, shouldConstructAndDestroyEmptyInstance)
{
   CIMUVN100T *imu = new CIMUVN100T();
   delete imu; imu = nullptr;
}

TEST(CIMUVN100T, shouldNotInitialize)
{
   CIMUVN100T *imu = new CIMUVN100T();

   imu->initialize();
   EXPECT_TRUE(imu->getState() == CGenericSensor::ssError);

   delete imu; imu = nullptr;
}

TEST(CIMUVN100T, shouldInitializeAndConnect)
{
   std:: cout << "Opening file: " << MRPT_VN100T_CONF_FILE << std::endl;
   CConfigFile iniFile(MRPT_VN100T_CONF_FILE);
   std::vector<std::string> sections;
   iniFile.getAllSections(sections);
   std::string sensorLabel= sections[1];
   std::string driverName = iniFile.read_string(sensorLabel, "driver", "", true);
   CGenericSensorPtr sensor = CGenericSensor::createSensorPtr(driverName);
   sensor->enableVerbose(false);

   if(!sensor)
   {
      throw std::logic_error("ProducerSensor::open - invalid sensor driver");
   }

   sensor->loadConfig(iniFile, sensorLabel);
   sensor->initialize();

   EXPECT_TRUE(sensor->getState() == CGenericSensor::ssWorking);
}

TEST(CIMUVN100T, shouldDoProcess)
{
   std:: cout << "Opening file: " << MRPT_VN100T_CONF_FILE << std::endl;
   CConfigFile iniFile(MRPT_VN100T_CONF_FILE);
   std::vector<std::string> sections;
   iniFile.getAllSections(sections);
   std::string sensorLabel= sections[1];
   std::string driverName = iniFile.read_string(sensorLabel, "driver", "", true);
   CGenericSensorPtr sensor = CGenericSensor::createSensorPtr(driverName);
   sensor->enableVerbose(false);

   if(!sensor)
   {
      throw std::logic_error("ProducerSensor::open - invalid sensor driver");
   }

   sensor->loadConfig(iniFile, sensorLabel);
   sensor->initialize();

   ASSERT_TRUE(sensor->getState() == CGenericSensor::ssWorking);

   sensor->doProcess();

   static CGenericSensor::TListObservations lstObjs;
   sensor->getObservations(lstObjs);

   ASSERT_GT(lstObjs.size(), 0);

   CGenericSensor::TListObservations::iterator it = lstObjs.begin();
   CObservationPtr obs = CObservationPtr(it->second);
   ASSERT_TRUE(IS_CLASS(CObservationPtr(obs), CObservationIMU));

   CObservationIMUPtr imuObs(obs);
   const double roll    = imuObs->rawMeasurements[IMU_ROLL];
   const double pitch   = imuObs->rawMeasurements[IMU_PITCH];
   const double yaw     = imuObs->rawMeasurements[IMU_YAW];

   std::cout << "Roll/Pitch/Yaw: " << roll << "/" << pitch << "/" << yaw << std::endl;

   EXPECT_NE(roll + pitch + yaw, 0);
}
