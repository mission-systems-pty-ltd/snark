set( KIT geometry )
file( GLOB source ${SOURCE_CODE_BASE_DIR}/math/${KIT}/test/*test.cpp )
set( test_name ${CMAKE_PROJECT_NAME}_test_math_${KIT} )
add_executable( ${test_name} ${source} )
target_link_libraries( ${test_name} snark_math_geometry ${GTEST_BOTH_LIBRARIES} pthread )
add_test( NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
if( INSTALL_TESTS )
    install( TARGETS ${test_name} RUNTIME DESTINATION ${snark_CPP_TESTS_INSTALL_DIR} COMPONENT Runtime )
    #INSTALL (
    #    FILES ${PROJECT_BINARY_DIR}/bin/${CMAKE_PROJECT_NAME}_test_${KIT}
    #    PERMISSIONS WORLD_READ GROUP_READ OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
    #    DESTINATION ${snark_CPP_TESTS_INSTALL_DIR} )
endif( INSTALL_TESTS )
