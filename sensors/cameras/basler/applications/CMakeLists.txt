source_group( basler-cat FILES basler-cat.cpp )

set( BINARY_DIR "${CMAKE_BINARY_DIR}/sensors/cameras/basler/applications" )

add_custom_target( basler.conf ) # quick and dirty
add_custom_command( TARGET basler.conf
                    POST_BUILD
                    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/basler-generate-ld-conf ARGS "${BASLER_PYLON_DIR}" )
install( FILES ${BINARY_DIR}/basler.conf DESTINATION /etc/ld.so.conf.d )
install( CODE "message( \"sensors/cameras/basler: running ldconfig\" )" )
install( CODE "execute_process( COMMAND ldconfig )" )

add_executable( basler-cat basler-cat.cpp )
add_dependencies( basler-cat basler.conf )

target_link_libraries( basler-cat ${snark_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} tbb pylonbase pylonutility )
set_target_properties( basler-cat PROPERTIES LINK_FLAGS_RELEASE -s )
if( ${BASLER_PYLON_MAJOR_VERSION} LESS 5 )
    target_link_libraries( basler-cat
        pylongigesupp
        GCBase_gcc40_v${BASLER_PYLON_MAJOR_VERSION}_${BASLER_PYLON_MINOR_VERSION} 
        GenApi_gcc40_v${BASLER_PYLON_MAJOR_VERSION}_${BASLER_PYLON_MINOR_VERSION} )
elseif( ${BASLER_PYLON_MAJOR_VERSION} EQUAL 5 )
#     message( "BASLER_PYLON_MAJOR_VERSION 5 or higher: ${BASLER_PYLON_MAJOR_VERSION}" )
    target_link_libraries( basler-cat
        pylon_TL_gige 
        GCBase_gcc_v3_0_Basler_pylon_v${BASLER_PYLON_MAJOR_VERSION}_${BASLER_PYLON_MINOR_VERSION} 
        GenApi_gcc_v3_0_Basler_pylon_v${BASLER_PYLON_MAJOR_VERSION}_${BASLER_PYLON_MINOR_VERSION} )
else()
    target_link_libraries( basler-cat
        pylon_TL_gige 
        GCBase_gcc_v3_1_Basler_pylon 
        GenApi_gcc_v3_1_Basler_pylon )
endif()

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBASLER_PYLON_DIR=\"${BASLER_PYLON_DIR}\"" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBASLER_PYLON_GENICAM_DIR=\"${BASLER_PYLON_GENICAM_DIR}\"" )

install( TARGETS basler-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
install( PROGRAMS basler-log basler-log-summary DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
