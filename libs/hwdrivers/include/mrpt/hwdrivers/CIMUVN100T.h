/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CIMUVN100T_H
#define CIMUVN100T_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
   namespace hwdrivers
   {

      /** @brief A class for interfacing VN-100-T IMU/AHRS, specifically the rugged model.
        *  The digital interface could be a serial TTL or RS-232, so no special drivers are needed.
        *
        *  See also the application "rawlog-grabber" for a ready-to-use application to gather data
        *  from the scanner.
        *
        *  \code
        *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
        * -------------------------------------------------------
        *   [supplied_section_name]
        *   pose_x=0                      ; Sensor 3D position relative to the robot (meters)
        *   pose_y=0
        *   pose_z=0
        *   pose_yaw=0                    ; Angles in degrees
        *   pose_pitch=0
        *   pose_roll=0
        *   sensorLabel = <label>         ; Label of the sensor
        *   COM_port_LIN = /dev/ttyUSB0   ; COM PORT in LINUX (If not provided, auto search)
        *   COM_port_WIN = COM1           ; COM PORT in Windows (If not provided, auto search)
        *   baudRate                      ; Baudrate for communicating with the COM port
        *   (Linux : Mandatory; Windows : if COM_port_WIN is not provided, this value is ignored)
        *  \endcode
        *  \ingroup mrpt_hwdrivers_grp
        */
      class HWDRIVERS_IMPEXP CIMUVN100T : public hwdrivers::CGenericSensor
      {
         DEFINE_GENERIC_SENSOR(CIMUVN100T)

      protected:
         int                        m_COMbauds;
         std::string                m_com_port;
         mrpt::system::TTimeStamp   m_timeStartTT;

         mrpt::poses::CPose3D       m_sensorPose;

         /** Set the port where the sensor is located and connect to it
           */
         bool connect();

         bool                       m_isConnected;
         void * /*Vn100 */          m_vn100_ptr;
         void * /*VnYpr */          m_ypr_ptr;
         unsigned int               m_toutCounter;    //!< Timeout counter (for internal use only)

         /** See the class documentation at the top for expected parameters */
         void loadConfig_sensorSpecific(
               const mrpt::utils::CConfigFileBase &configSource,
               const std::string	&iniSection);

      public:
         /** Constructor
           */
         CIMUVN100T();

         /** Destructor
           */
         virtual ~CIMUVN100T();

         /** This method will be invoked at a minimum rate of "process_rate" (Hz)
           *  \exception This method must throw an exception with a descriptive message if some critical error is found.
           */
         void doProcess() override;

         /** Turns on the xSens device and configure it for getting orientation data */
         void initialize() override;
      };
   }
}

#endif // CIMUVN100T_H
