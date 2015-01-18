/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/threads.h>
#include <mrpt/hwdrivers/CIMUVN100T.h>
#include <mrpt/obs/CObservationIMU.h>

IMPLEMENTS_GENERIC_SENSOR(CIMUVN100T,mrpt::hwdrivers)

using namespace mrpt::utils;
using namespace mrpt::obs;

#if MRPT_HAS_VECTORNAV
   #include "vectornav/vectornav.h"
#endif

// Adaptors for the "void*" memory blocks:
#define vn100  (*static_cast<Vn100*>(m_vn100_ptr))
#define ypr    (*static_cast<VnYpr*>(m_ypr_ptr))

// Include libraries in linking:
#if MRPT_HAS_VECTORNAV
   #ifdef MRPT_OS_WINDOWS
      // WINDOWS:
      #if defined(_MSC_VER) || defined(__BORLANDC__)
         #pragma comment (lib,"SetupAPI.lib")
      #endif
   #endif   // MRPT_OS_WINDOWS
#endif   // MRPT_HAS_VECTORNAV

namespace mrpt
{
namespace hwdrivers
{

/*-------------------------------------------------------------
               CIMUVN100T
-------------------------------------------------------------*/
CIMUVN100T::CIMUVN100T() :
   m_COMbauds(0),
   m_com_port(),
   m_timeStartTT(0),
   m_sensorPose(),
   m_isConnected(false),
   m_vn100_ptr(NULL),
   m_ypr_ptr(NULL),
   m_toutCounter(0)
{
   m_sensorLabel = "VN100T";
#if MRPT_HAS_VECTORNAV
    m_vn100_ptr   = new Vn100[1];
    m_ypr_ptr     = new VnYpr[1];
#else
   THROW_EXCEPTION("MRPT has been compiled with 'BUILD_VN100T'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
               ~CIMUVN100T
-------------------------------------------------------------*/
CIMUVN100T::~CIMUVN100T()
{
#if MRPT_HAS_VECTORNAV
   if(m_isConnected)
   {
      VN_ERROR_CODE errorCode = vn100_disconnect(&vn100);
      m_isConnected = false;
      if(errorCode != VNERR_NO_ERROR)
      {
         std::cerr << "Error encountered when trying to disconnect from the sensor." << std::endl;
      }

      delete[] &vn100;  m_vn100_ptr = NULL;
      delete[] &ypr;    m_ypr_ptr = NULL;
   }
#endif
}

/*-------------------------------------------------------------
               doProcess
-------------------------------------------------------------*/
void CIMUVN100T::doProcess()
{
#if MRPT_HAS_VECTORNAV

   if(m_state == ssError)
   {
      mrpt::system::sleep(200);
      initialize();
   }

   if(m_state == ssError)
      return;

   VN_ERROR_CODE errorCode;
   unsigned int cont = 0;

   do
   {
      errorCode = vn100_getYawPitchRoll(&vn100, &ypr);

      if(errorCode == VNERR_NO_ERROR)
      {
         // Data properly collected
         m_state = ssWorking;

         CObservationIMUPtr obs = CObservationIMU::Create();

         // ANGLE MEASUREMENTS: minus to get NWU system
         obs->rawMeasurements[IMU_YAW]    = -DEG2RAD(ypr.yaw);
         obs->dataIsPresent[IMU_YAW]      = true;
         obs->rawMeasurements[IMU_PITCH]  = -DEG2RAD(ypr.pitch);
         obs->dataIsPresent[IMU_PITCH]    = true;
         obs->rawMeasurements[IMU_ROLL]   = DEG2RAD(ypr.roll);
         obs->dataIsPresent[IMU_ROLL]     = true;

         // ACCELEROMETERS MEASUREMENTS:
         obs->dataIsPresent[IMU_X_ACC]	= false;
         obs->dataIsPresent[IMU_Y_ACC]	= false;
         obs->dataIsPresent[IMU_Z_ACC]	= false;

         // GYROSCOPES MEASUREMENTS:
         obs->dataIsPresent[IMU_YAW_VEL]     = false;
         obs->dataIsPresent[IMU_PITCH_VEL]   = false;
         obs->dataIsPresent[IMU_ROLL_VEL]    = false;

         // TimeStamp
         obs->timestamp    = mrpt::system::now();
         obs->sensorPose   = m_sensorPose;
         obs->sensorLabel  = m_sensorLabel;

         appendObservation(obs);
         m_toutCounter = 0;
      }

      if(errorCode == VNERR_TIMEOUT)
      {
         if(++m_toutCounter > 3)
         {
            m_toutCounter  = 0;
            m_state        = ssError;
            if(m_isConnected)
            {
               vn100_disconnect(&vn100);
               m_isConnected = false;
            }

            std::cerr << "[CIMUVN100T::doProcess()] Error: No data available [VNERR_TIMEOUT]" << std::endl;
         }
      }

   } while(errorCode == VNERR_NO_ERROR && cont++ < 30);

#else
   THROW_EXCEPTION("MRPT has been compiled with 'BUILD_VN100T'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
               connect
-------------------------------------------------------------*/
bool CIMUVN100T::connect()
{
#if MRPT_HAS_VECTORNAV
   VN_ERROR_CODE errorCode;

   // Port defined by user in .ini file
   std::cout << "Using COM port " << m_com_port << " at " << m_COMbauds << " baud" << std::endl;
   std::cout << "Opening port..." << std::endl;

   //open the port which the device is connected to and connect at the device's baudrate.
   errorCode = vn100_connect(&vn100, m_com_port.c_str(), m_COMbauds);

   /* Make sure the user has permission to use the COM port. */
   if(errorCode == VNERR_PERMISSION_DENIED)
   {
      std::cerr << "Current user does not have permission to open the COM port.\n"
                << "Try running again using 'sudo'." << std::endl;
      m_state = ssError;
      return false;
   }
   else if (errorCode != VNERR_NO_ERROR)
   {
      std::cerr << "Error encountered when trying to connect to the sensor." << std::endl;
      m_state = ssError;
      return false;
   }
   std::cout << "done" << std::endl;

   m_isConnected = true;

   return true;
#else
   return false;
#endif
}

/*-------------------------------------------------------------
               initialize
-------------------------------------------------------------*/
void CIMUVN100T::initialize()
{
#if MRPT_HAS_VECTORNAV

   VN_ERROR_CODE errorCode;

   if(m_isConnected)
      return;

   m_state = ssInitializing;

   // Search for the COM PORT and connect
   if(!connect())
   {
      m_state = ssError;
      std::cerr << "Error Could not initialize the device" << std::endl;
      return;
   }

   std::cout << "VN100T IMU detected and connected" << std::endl;

   std::cout << "Getting initial TimeStamp" << std::endl;
   // Get initial TimeStamp
   do
   {
      errorCode = vn100_getYawPitchRoll(&vn100, &ypr);
      if (errorCode == VNERR_NO_ERROR)
      {
         m_timeStartTT = mrpt::system::now();
      }
   } while(errorCode != VNERR_NO_ERROR);

   std::cout << "Gathering data" << std::endl;
   m_state = ssWorking;

#else
   THROW_EXCEPTION("MRPT has been compiled with 'BUILD_VN100T'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
               loadConfig_sensorSpecific
-------------------------------------------------------------*/
void CIMUVN100T::loadConfig_sensorSpecific(
      const mrpt::utils::CConfigFileBase &configSource,
      const std::string	&iniSection)
{
   m_sensorPose.setFromValues(
        configSource.read_float(iniSection, "pose_x", 0, false),
        configSource.read_float(iniSection, "pose_y", 0, false),
        configSource.read_float(iniSection, "pose_z", 0, false),
        DEG2RAD(configSource.read_float(iniSection, "pose_yaw", 0, false)),
        DEG2RAD(configSource.read_float(iniSection, "pose_pitch", 0, false)),
        DEG2RAD(configSource.read_float(iniSection, "pose_roll", 0, false)));

   m_COMbauds = configSource.read_int(iniSection, "baudRate", m_COMbauds, false);

#ifdef MRPT_OS_WINDOWS
   m_com_port = configSource.read_string(iniSection, "COM_port_WIN", m_com_port, false);
#else
   m_com_port = configSource.read_string(iniSection, "COM_port_LIN", m_com_port, false);
#endif
}

}
}
