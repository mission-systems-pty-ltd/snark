SET( PROJECT "render" )
SET( TARGET_NAME snark_${PROJECT} )

SET( dir ${SOURCE_CODE_BASE_DIR}/${PROJECT} )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h )
FILE( GLOB colours_source   ${dir}/colours/*.cpp)
FILE( GLOB colours_includes ${dir}/colours/*.h )

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} ${colours_source} ${colours_includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
set_target_properties( ${TARGET_NAME} PROPERTIES LINK_FLAGS_RELEASE -s )
TARGET_LINK_LIBRARIES( ${TARGET_NAME} ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
INSTALL( FILES ${colours_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/colours )
INSTALL( TARGETS ${TARGET_NAME}
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime
         LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime
         ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development )

ADD_SUBDIRECTORY( kml )
ADD_SUBDIRECTORY( svg )
