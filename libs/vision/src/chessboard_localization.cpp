/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/vision/chessboard_localization.h>

#include <mrpt/vision/pinhole.h>
#include <mrpt/math/robust_kernels.h>
#include <mrpt/math/wrap2pi.h>

//#include "chessboard_stereo_camera_calib_internal.h"

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;


TChessboardLocalizationParams::TChessboardLocalizationParams()  :
	corners_nx(0), corners_ny(0),
	init_pose(NULL),
	robust(true),
	robust_kernel_param(3)
{
}

double mrpt::vision::chessboard_localization(
	const std::vector<mrpt::utils::TPixelCoordf> &detected_corners, 
	mrpt::poses::CPose3D & out_cam_pose,
	const TChessboardLocalizationParams &params
	)
{
	MRPT_TODO("xx")

	return 0;
}
