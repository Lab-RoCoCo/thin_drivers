# - Try to find libfreenect_sync
FIND_PATH(
  libfreenectsync_INCLUDE_DIRS
  NAMES libfreenect_sync.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../libfreenect/wrappers/c_sync
    /usr/include
    /usr/local/include
    /usr/include/libfreenect
    /usr/local/include/libfreenect
)

FIND_LIBRARY(
  libfreenectsync_LIBRARY
  NAMES freenect_sync
  PATHS
    ${CMAKE_SOURCE_DIR}/../libfreenect/build/lib
    /usr/lib
    /usr/local/lib
)

IF(libfreenectsync_INCLUDE_DIRS AND libfreenectsync_LIBRARY)
  SET(libfreenectsync_FOUND TRUE)
ENDIF()
