# Build the xSENS support in mrpt-hwdrivers?
# ===================================================

SET(BUILD_VN100T ON CACHE BOOL "Build VectorNav VN100T libraries")

# Create config vars for VECTORNAV:
SET(CMAKE_MRPT_HAS_VECTORNAV 0)
IF(BUILD_VN100T)
   SET(CMAKE_MRPT_HAS_VECTORNAV 1)
ENDIF(BUILD_VN100T)
