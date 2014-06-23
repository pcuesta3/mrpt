/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/utils/TPixelCoord.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/vision/link_pragmas.h>

namespace mrpt
{
	namespace vision
	{
		/** \addtogroup chessboard_calib Chessboard calibration
		  *  \ingroup mrpt_vision_grp
		  *  @{  */

		struct VISION_IMPEXP TChessboardLocalizationParams
		{
			mrpt::utils::TCamera   cam_params;             //!< Known camera parameters (from calibration)
			unsigned int           corners_nx, corners_ny; //!< Number of corners in the X/Y direction (see picture in mrpt::vision::chessboard_localization for axes)
			mrpt::poses::CPose3D * init_pose;              //!< (Default=NULL) Optionally, an initial guess for the camera pose wrt the chessboard

			bool robust;  //!< Use robust kernel (default=true)
			double robust_kernel_param; //!< Default = 3 (in pixels)

			TChessboardLocalizationParams();
		};

		/** Least-squares recovering of the SE(3) camera pose of known 
		  *
		  * \param[in] detected_corners Pixel coordinates of detected corners of a chessboard, in row(X)-major order. Number of points = corners_nx * corners_ny
		  * \param[in] params See struct for details
		  * \param[out] out_cam_pose The recovered camera pose after optimization (see picture for axes)
		  *
		  * \return The final RMSE error, in pixels
		  */
		double VISION_IMPEXP chessboard_localization(
			const std::vector<mrpt::utils::TPixelCoordf> &detected_corners, 
			mrpt::poses::CPose3D & out_cam_pose,
			const TChessboardLocalizationParams &params
			);

		/** @}  */ // end of grouping

	}
}
