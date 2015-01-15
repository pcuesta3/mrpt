# Check for the LAS (LiDAR format) library
# ===================================================

SET(CMAKE_MRPT_HAS_LIBLAS 0)

# Leave at the user's choice to disable the LAS lib:
OPTION(DISABLE_LIBLAS "Forces NOT using LAS lib, even if it could be found by CMake" "OFF")
MARK_AS_ADVANCED(DISABLE_LIBLAS)

IF(NOT DISABLE_LIBLAS)
   # Windows: (Not supported for now)
   IF(UNIX)
      FIND_FILE(LAS_CONFIG_FILE liblas-config)
      IF(LAS_CONFIG_FILE)
         MARK_AS_ADVANCED(LAS_CONFIG_FILE)

         SET(CMAKE_MRPT_HAS_LIBLAS 1)
         SET(CMAKE_MRPT_HAS_LIBLAS_SYSTEM 1)

         # Get the config params:
         EXECUTE_PROCESS(COMMAND ${LAS_CONFIG_FILE} --libs
            RESULT_VARIABLE CMAKE_LAS_CONFIG_RES
            OUTPUT_VARIABLE CMAKE_LAS_LIBS
            OUTPUT_STRIP_TRAILING_WHITESPACE
            )
         IF(${CMAKE_LAS_CONFIG_RES})
            MESSAGE("Error invoking LAS config file:\n ${LAS_CONFIG_FILE} --libs")
         ENDIF(${CMAKE_LAS_CONFIG_RES})

         EXECUTE_PROCESS(COMMAND ${LAS_CONFIG_FILE} --includes
            RESULT_VARIABLE CMAKE_LAS_CONFIG_RES
            OUTPUT_VARIABLE CMAKE_LAS_INCLUDES
            OUTPUT_STRIP_TRAILING_WHITESPACE
            )
         IF(${CMAKE_LAS_CONFIG_RES})
            MESSAGE("Error invoking LAS config file:\n ${LAS_CONFIG_FILE} --includes")
         ENDIF(${CMAKE_LAS_CONFIG_RES})

         # Join all flags and parse to separate them:
         SET(CMAKE_LAS_CFGS "${CMAKE_LAS_LIBS} ${CMAKE_LAS_INCLUDES}")

         pkgconfig_parse(${CMAKE_LAS_CFGS} "LAS")

         # For some reason, "liblas-config --libs" return all other libs, except liblas itself:
         LIST(APPEND LAS_LIBS "las")

         IF($ENV{VERBOSE})
            MESSAGE(STATUS "liblas configuration:")
            MESSAGE(STATUS "  LAS_INCLUDE_DIRS: ${LAS_INCLUDE_DIRS}")
            MESSAGE(STATUS "  LAS_LINK_DIRS: ${LAS_LINK_DIRS}")
            MESSAGE(STATUS "  LAS_LIBS: ${LAS_LIBS}")
         ENDIF($ENV{VERBOSE})

      ENDIF(LAS_CONFIG_FILE)
   ENDIF(UNIX)
ENDIF(NOT DISABLE_LIBLAS)

