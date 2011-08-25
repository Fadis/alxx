CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/al++/config.hpp.in ${CMAKE_CURRENT_BINARY_DIR}/al++/config.hpp)

INSTALL( DIRECTORY al++/
         DESTINATION include/al++/
         FILES_MATCHING PATTERN "*.hpp" )
