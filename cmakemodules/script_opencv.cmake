# Check for the OpenCV libraries:
#  pkg-config if available (Linux), otherwise CMake module
# =========================================================
SET(CMAKE_MRPT_HAS_OPENCV 0)
SET(MRPT_OPENCV_VERSION 0.0.0)
SET(MRPT_OPENCV_VERSION_HEX "0x000")
SET(MRPT_OPENCV_SRC_DIR "")
SET(OpenCV_IGNORE_PKGCONFIG OFF CACHE BOOL "Forces using OpenCVConfig.cmake to find OpenCV")
MARK_AS_ADVANCED(OpenCV_IGNORE_PKGCONFIG)

# Use CMAKE module if opencv's not been detected yet:
IF(NOT CMAKE_MRPT_HAS_OPENCV)
	# 1st: Try to find OpenCV config file (NO_MODULE: Don't find a module, but OpenCVConfig.cmake):
   FIND_PACKAGE(OpenCV  2.4 QUIET NO_MODULE)
	IF(OpenCV_FOUND)
		SET(MRPT_OPENCV_VERSION ${OpenCV_VERSION})
		SET(OpenCV_LIBRARIES ${OpenCV_LIBS})
		SET(OPENCV_LIBDIR ${OpenCV_LIB_DIR})
		if (NOT "${BASEDIR}" STREQUAL "")
			SET(MRPT_OPENCV_SRC_DIR "${BASEDIR}")
		endif (NOT "${BASEDIR}" STREQUAL "")
		IF($ENV{VERBOSE})
			MESSAGE(STATUS "OpenCV ${OpenCV_VERSION} found through OpenCVConfig.cmake")
		ENDIF($ENV{VERBOSE})

		SET(CMAKE_MRPT_HAS_OPENCV 1)
	ENDIF(OpenCV_FOUND)
ENDIF(NOT CMAKE_MRPT_HAS_OPENCV)

# 2nd: Invoke pkg-config for getting the configuration:
IF(NOT CMAKE_MRPT_HAS_OPENCV AND PKG_CONFIG_FOUND AND NOT OpenCV_IGNORE_PKGCONFIG)
	PKG_CHECK_MODULES(OPENCV ${_QUIET} opencv)
	IF(OPENCV_FOUND)
		SET(CMAKE_MRPT_HAS_OPENCV 1)
		SET(MRPT_OPENCV_VERSION ${OPENCV_VERSION})

		IF ("${OPENCV_LIBDIR}")
			LINK_DIRECTORIES(${OPENCV_LIBDIR})
		ENDIF ("${OPENCV_LIBDIR}")
		SET(OpenCV_LIBRARIES ${OPENCV_LIBRARIES})

		IF($ENV{VERBOSE})
			MESSAGE(STATUS " opencv include: ${OPENCV_INCLUDE_DIRS} (Version: ${OPENCV_VERSION})")
		ENDIF($ENV{VERBOSE})
	ENDIF(OPENCV_FOUND)
ENDIF(NOT CMAKE_MRPT_HAS_OPENCV AND PKG_CONFIG_FOUND AND NOT OpenCV_IGNORE_PKGCONFIG)


IF(NOT CMAKE_MRPT_HAS_OPENCV)
	# 3rd: OK, let's use the module:
	FIND_PACKAGE(OpenCV)
	IF(OpenCV_FOUND)
		# MRPT_OPENCV_VERSION
		IF($ENV{VERBOSE})
			MESSAGE(STATUS "OPENCV_EXE_LINKER_FLAGS: ${OpenCV_EXE_LINKER_FLAGS}")
			MESSAGE(STATUS "OPENCV_INCLUDE_DIR: ${OpenCV_INCLUDE_DIR}")
			MESSAGE(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
		ENDIF($ENV{VERBOSE})

		FILE(GLOB_RECURSE CV_VER_H "${OpenCV_CXCORE_INCLUDE_DIR}/cvver.h")
		file(READ "${CV_VER_H}" STR_CV_VERSION)

		# Extract the CV version from the cvver.h file, lines "#define CV_MAJOR_VERSION  XX", etc...

		#STRING(REGEX MATCHALL "[0-9]+.[0-9]+.[0-9]+" MRPT_OPENCV_VERSION "${STR_CV_VERSION}")
		STRING(REGEX MATCH "CV_MAJOR_VERSION[ ]+[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MAJOR "${STR_CV_VERSION}")
		STRING(REGEX MATCH "[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MAJOR "${CMAKE_OPENCV_VERSION_NUMBER_MAJOR}")

		STRING(REGEX MATCH "CV_MINOR_VERSION[ ]+[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MINOR "${STR_CV_VERSION}")
		STRING(REGEX MATCH "[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MINOR "${CMAKE_OPENCV_VERSION_NUMBER_MINOR}")

		STRING(REGEX MATCH "CV_SUBMINOR_VERSION[ ]+[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_PATCH "${STR_CV_VERSION}")
		STRING(REGEX MATCH "[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_PATCH "${CMAKE_OPENCV_VERSION_NUMBER_PATCH}")

		SET(MRPT_OPENCV_VERSION "${CMAKE_OPENCV_VERSION_NUMBER_MAJOR}.${CMAKE_OPENCV_VERSION_NUMBER_MINOR}.${CMAKE_OPENCV_VERSION_NUMBER_PATCH}")

		IF($ENV{VERBOSE})
			MESSAGE(STATUS "OpenCV version detected: ${MRPT_OPENCV_VERSION}")
		ENDIF($ENV{VERBOSE})

		SET(CMAKE_MRPT_HAS_OPENCV 1)
	ENDIF(OpenCV_FOUND)
ENDIF(NOT CMAKE_MRPT_HAS_OPENCV)


# Opencv version as Hex. number:
VERSION_TO_HEXADECIMAL(MRPT_OPENCV_VERSION_HEX ${MRPT_OPENCV_VERSION})

# DISABLE_OPENCV
# ---------------------
OPTION(DISABLE_OPENCV "Disable the OpenCV library" "OFF")
MARK_AS_ADVANCED(DISABLE_OPENCV)
IF(DISABLE_OPENCV)
	SET(CMAKE_MRPT_HAS_OPENCV 0)
ENDIF(DISABLE_OPENCV)


# OpenCV (all compilers):
SET(CMAKE_MRPT_HAS_OPENCV_NONFREE 0)
IF(CMAKE_MRPT_HAS_OPENCV)
	# Important: we can't link against opencv_ts, apparently it leads to crashes 
	# when also linking to gtest (???)
	list(REMOVE_ITEM OpenCV_LIBRARIES opencv_ts)

	#INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${OPENCV_INCLUDEDIR})
	#APPEND_MRPT_LIBS(${OpenCV_LIBRARIES})

	IF($ENV{VERBOSE})
		MESSAGE(STATUS "OpenCV:")
		MESSAGE(STATUS "        OpenCV_LIBRARIES:   ${OpenCV_LIBRARIES}")
		MESSAGE(STATUS "        OpenCV_INCLUDE_DIR: ${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS}")
	ENDIF($ENV{VERBOSE})

	IF ("${OpenCV_LIBRARIES}" MATCHES ".*opencv_nonfree.*")
		SET(CMAKE_MRPT_HAS_OPENCV_NONFREE 1)
	ENDIF ("${OpenCV_LIBRARIES}" MATCHES ".*opencv_nonfree.*")
	

	# Add OpenCV directories as "-isystem" to avoid warnings with :
	ADD_DIRECTORIES_AS_ISYSTEM(OpenCV_INCLUDE_DIRS)

	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OPENCV_EXE_LINKER_FLAGS}")

	SET(CMAKE_MRPT_HAS_OPENCV_SYSTEM 1)

ENDIF(CMAKE_MRPT_HAS_OPENCV)
