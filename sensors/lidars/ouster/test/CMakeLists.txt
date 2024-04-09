# Copyright (c) 2024 Mission Systems Pty Ltd

set( KIT ouster )
set( test_name ${CMAKE_PROJECT_NAME}_test_sensors_lidars_${KIT} )
file( GLOB source ${SOURCE_CODE_BASE_DIR}/sensors/lidars/${KIT}/test/*.cpp )
add_executable( ${test_name} ${source} )
target_link_libraries( ${test_name} snark_ouster GTest::gtest )
add_test( NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
if( INSTALL_TESTS )
    install( TARGETS ${test_name} RUNTIME DESTINATION ${snark_CPP_TESTS_INSTALL_DIR} COMPONENT Runtime )
endif( INSTALL_TESTS )
