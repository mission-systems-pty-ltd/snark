SET( snark_actuators_dir ${SOURCE_CODE_BASE_DIR}/actuators )

add_subdirectory( applications )

IF( snark_build_actuators_quickset )
    ADD_SUBDIRECTORY( quickset )
ENDIF( snark_build_actuators_quickset )

if( snark_build_actuators_wheels )
    add_subdirectory( wheels )
endif( snark_build_actuators_wheels )

add_subdirectory( universal_robots )
