# PTP support
#
# PTP is only supported on GigE cameras. Normally it would be activated or not
# at run-time however the code to display PTP stats inside vimba-cat indirectly
# uses libboost_filesystem which leads to library compatibility issues with the
# CSI transport layer CTI binary, which dynamically links against
# libboost_filesystem and expects to see version 1.58 exactly. On ARM this leads
# to a segfault.
#
# Therefore we make PTP support a compile-time option which, so we don't link to
# libboost_filesystem at all in the application code if we don't need to.
#
# Test with:
# $ vimba-cat --list-cameras
#
# See Alvium Features Reference for details of PTP support by camera.

set( snark_vimba_cat_ptp_support ON CACHE BOOL "build vimba-cat with PTP support" )

source_group( vimba-cat FILES vimba-cat.cpp )
# specifically add serialization.cpp, rather than linking against snark_imaging,
# to avoid pulling in boost filesystem unnecessarily
add_executable( vimba-cat vimba-cat.cpp ../../../../imaging/cv_mat/serialization.cpp )
if( snark_vimba_cat_ptp_support )
    set_source_files_properties( vimba-cat.cpp PROPERTIES COMPILE_OPTIONS "-DVIMBA_PTP_SUPPORT" )
endif()
# we also selectively choose which comma libraries to link against, also to avoid boost filesystem
set( vimba_comma_libraries comma_application comma_string comma_csv comma_xpath comma_name_value comma_base )
if( snark_vimba_cat_ptp_support )
    set( vimba_comma_libraries comma_io ${vimba_comma_libraries} )
endif()
target_link_libraries( vimba-cat snark_vimba ${OpenCV_LIBS} VimbaCPP ${vimba_comma_libraries} )
set_target_properties( vimba-cat PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS vimba-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
