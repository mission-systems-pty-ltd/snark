SET( dir ${SOURCE_CODE_BASE_DIR}/math/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( math-deg2rad math-deg2rad.cpp )
TARGET_LINK_LIBRARIES( math-deg2rad ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( math-deg2rad PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS math-deg2rad RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( math-rad2deg math-rad2deg.cpp )
TARGET_LINK_LIBRARIES( math-rad2deg ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( math-rad2deg PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS math-rad2deg RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

IF( CMAKE_SIZEOF_VOID_P EQUAL 4 )
    # disable eigen vectorization on 32 bit machines
    # see http://eigen.tuxfamily.org/index.php?title=FAQ#Vectorization
    ADD_DEFINITIONS( -DEIGEN_DONT_VECTORIZE )
    ADD_DEFINITIONS( -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT )
    ADD_DEFINITIONS( -DEIGEN_DONT_ALIGN_STATICALLY )
ENDIF()

ADD_EXECUTABLE( points-frame frame.cpp points-frame.cpp )
TARGET_LINK_LIBRARIES( points-frame snark_math ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( points-frame PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS points-frame RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( points-to-cartesian points-to-cartesian.cpp )
TARGET_LINK_LIBRARIES( points-to-cartesian snark_math ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( points-to-cartesian PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS points-to-cartesian RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( points-to-polar points-to-polar.cpp )
TARGET_LINK_LIBRARIES( points-to-polar snark_math ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( points-to-polar PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS points-to-polar RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( ellipsoid-calc ellipsoid-calc.cpp )
TARGET_LINK_LIBRARIES( ellipsoid-calc snark_math_spherical_geometry ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( ellipsoid-calc PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS ellipsoid-calc RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

add_executable( math-eigen math-eigen.cpp )
target_link_libraries( math-eigen snark_math ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( math-eigen PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS math-eigen RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

if( USE_CUDA )
    cuda_add_executable( math-k-means math-k-means.cpp math-k-means/math_k_means.cu )
else()
    add_executable( math-k-means math-k-means.cpp )
endif()

target_link_libraries( math-k-means snark_math ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} tbb )
set_target_properties( math-k-means PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS math-k-means RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

if(snark_build_math_fft)
    add_executable( math-fft math-fft.cpp fft/basic.h fft/basic.cpp )
    target_link_libraries( math-fft ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} fftw3 )
    set_target_properties( math-fft PROPERTIES LINK_FLAGS_RELEASE -s )
    install( TARGETS math-fft RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
endif(snark_build_math_fft)

install( PROGRAMS math-matlab-convert DESTINATION ${snark_INSTALL_BIN_DIR} )
install( PROGRAMS math-array DESTINATION ${snark_INSTALL_BIN_DIR} )

