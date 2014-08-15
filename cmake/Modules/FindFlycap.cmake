# - Try to find Flycap
# Once done this will define
#
#  FLYCAP_FOUND - system has Flycap
#  FLYCAP_INCLUDE_DIRS - the Flycap include directory
#  FLYCAP_LIBRARIES - Link these to use Flycap
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (FLYCAP_LIBRARIES AND FLYCAP_INCLUDE_DIRS)
  # in cache already
  set(FLYCAP_FOUND TRUE)
else (FLYCAP_LIBRARIES AND FLYCAP_INCLUDE_DIRS)

  find_path(FLYCAP_INCLUDE_DIR
    NAMES
      flycapture/FlyCapture2.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(FLYCAP_LIBRARY
    NAMES
      flycapture
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(FLYCAP_INCLUDE_DIRS
    ${FLYCAP_INCLUDE_DIR}
  )

  if (FLYCAP_LIBRARY)
    set(FLYCAP_LIBRARIES
        ${FLYCAP_LIBRARIES}
        ${FLYCAP_LIBRARY}
    )
  endif (FLYCAP_LIBRARY)

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(Flycap DEFAULT_MSG FLYCAP_LIBRARIES FLYCAP_INCLUDE_DIRS)

  # show the FLYCAP_INCLUDE_DIRS and FLYCAP_LIBRARIES variables only in the advanced view
  mark_as_advanced(FLYCAP_INCLUDE_DIRS FLYCAP_LIBRARIES)

endif (FLYCAP_LIBRARIES AND FLYCAP_INCLUDE_DIRS)

