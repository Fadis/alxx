add_library( al++ SHARED al++.cpp )
target_link_libraries( al++ ${Boost_LIBRARIES} ${OPENAL_LIBRARY} ${TBB_LIBRARIES} )

SET_TARGET_PROPERTIES( al++
                       PROPERTIES
                       VERSION 1.0.0
                       SOVERSION 1 )

INSTALL(TARGETS al++
        DESTINATION lib
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ)

subdirs( examples )
