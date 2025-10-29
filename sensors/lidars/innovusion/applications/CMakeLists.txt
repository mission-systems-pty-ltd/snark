string( REGEX MATCH "17" version_match ${CXX_STANDARD_TO_USE} )
if( "${version_match}" STREQUAL "" )
    message( FATAL_ERROR "\ninnovusion-cat requires C++17\n"
        "Set CXX_STANDARD_TO_USE (or disable this module)"
        )
endif()

foreach( innovusion_build_version IN LISTS snark_build_sensors_innovusion_sdk_versions )
    message( STATUS "build innovusion-cat version ${innovusion_build_version}" )

    set( application_name innovusion-cat${innovusion_build_version} )

    source_group( ${application_name} FILES innovusion-cat.cpp )
    add_executable( ${application_name} innovusion-cat.cpp )
    set_target_properties( ${application_name} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE )
    set_target_properties( ${application_name} PROPERTIES LINK_FLAGS_RELEASE -s )
    target_link_libraries( ${application_name} PRIVATE snark_innovusion${innovusion_build_version} )
    target_include_directories( ${application_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/v${innovusion_build_version} )

    install( TARGETS ${application_name}
             RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
             COMPONENT Runtime )
endforeach()
