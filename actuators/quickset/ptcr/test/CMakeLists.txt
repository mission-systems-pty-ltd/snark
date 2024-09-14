SET( KIT quickset_ptcr )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/actuators/quickset/ptcr/test/*test.cpp )
FILE( GLOB extras ${SOURCE_CODE_BASE_DIR}/actuators/quickset/ptcr/test/*.cpp
    ${SOURCE_CODE_BASE_DIR}/actuators/quickset/ptcr/test/*.h )
LIST( REMOVE_ITEM extras ${source} )
set( test_name "${CMAKE_PROJECT_NAME}_test_${KIT}" )
ADD_EXECUTABLE( ${test_name} ${source} ${extras} )
TARGET_LINK_LIBRARIES( ${test_name}
                       snark_quickset_ptcr
                       ${snark_ALL_EXTERNAL_LIBRARIES}
                       ${GTEST_BOTH_LIBRARIES} )
IF( INSTALL_TESTS )
	INSTALL( TARGETS ${test_name}
             PERMISSIONS WORLD_READ GROUP_READ OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
             DESTINATION ${snark_CPP_TESTS_INSTALL_DIR} )
ENDIF( INSTALL_TESTS )

add_test( NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
