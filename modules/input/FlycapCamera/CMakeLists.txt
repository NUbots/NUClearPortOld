# Find libjpeg library
FIND_PACKAGE(Flycap REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES ${FLYCAP_INCLUDE_DIR}
               LIBRARIES ${FLYCAP_LIBRARIES})
