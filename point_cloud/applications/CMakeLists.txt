set( dir ${SOURCE_CODE_BASE_DIR}/point_cloud/applications )
file( GLOB source   ${dir}/*.cpp)
file( GLOB includes ${dir}/*.h)
file( GLOB points_calc_source ${dir}/points_calc/*.cpp)
file( GLOB points_calc_includes ${dir}/points_calc/*.h)
file( GLOB points_grep_source ${dir}/points_grep/*.cpp)
file( GLOB points_grep_includes ${dir}/points_grep/*.h)
source_group( ${TARGET_NAME} FILES ${source} ${includes} ${points_calc_source} ${points_calc_includes} ${points_grep_includes} ${points_grep_source} )

add_executable( points-align points-align.cpp )
add_executable( points-calc points-calc.cpp ${points_calc_source} )
add_executable( points-detect-change points-detect-change.cpp )
add_executable( points-foreground-partitions points-foreground-partitions.cpp )
add_executable( points-make points-make.cpp )
add_executable( points-rays points-rays.cpp )
add_executable( points-to-centroids points-to-centroids.cpp )
add_executable( points-to-partitions points-to-partitions.cpp )
add_executable( points-to-voxels points-to-voxels.cpp )
add_executable( points-to-voxel-indices points-to-voxel-indices.cpp )
add_executable( points-track-partitions points-track-partitions.cpp )

target_link_libraries( points-align snark_math ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
target_link_libraries( points-calc snark_point_cloud snark_math snark_math_geometry ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
target_link_libraries( points-detect-change snark_math ${comma_ALL_LIBRARIES} ) #profiler )
target_link_libraries( points-foreground-partitions snark_point_cloud ${comma_ALL_LIBRARIES} tbb )
target_link_libraries( points-make snark_math ${comma_ALL_LIBRARIES} )
target_link_libraries( points-rays snark_math snark_math_spherical_geometry ${comma_ALL_LIBRARIES} tbb ) #profiler )
target_link_libraries( points-to-centroids snark_point_cloud ${comma_ALL_LIBRARIES} tbb )
target_link_libraries( points-to-partitions snark_point_cloud ${comma_ALL_LIBRARIES} tbb )
target_link_libraries( points-to-voxels snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
target_link_libraries( points-to-voxel-indices snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
target_link_libraries( points-track-partitions ${comma_ALL_LIBRARIES} )

set_target_properties( points-align PROPERTIES LINK_FLAGS_RELEASE -s )
set_target_properties( points-calc PROPERTIES LINK_FLAGS_RELEASE -s )
set_target_properties( points-detect-change PROPERTIES LINK_FLAGS_RELEASE -s )
set_target_properties( points-foreground-partitions PROPERTIES LINK_FLAGS_RELEASE -s )
set_target_properties( points-make PROPERTIES LINK_FLAGS_RELEASE -s )
set_target_properties( points-rays PROPERTIES LINK_FLAGS_RELEASE -s )
set_target_properties( points-to-centroids PROPERTIES LINK_FLAGS_RELEASE -s )
set_target_properties( points-to-partitions PROPERTIES LINK_FLAGS_RELEASE -s )
set_target_properties( points-to-voxels PROPERTIES LINK_FLAGS_RELEASE -s )
set_target_properties( points-to-voxel-indices PROPERTIES LINK_FLAGS_RELEASE -s )
set_target_properties( points-track-partitions PROPERTIES LINK_FLAGS_RELEASE -s )

if( PROFILE )
    target_link_libraries ( points-to-partitions profiler )
endif( PROFILE )

install( TARGETS points-align
                 points-calc
                 points-detect-change
                 points-foreground-partitions
                 points-make
                 points-rays
                 points-to-centroids
                 points-to-partitions
                 points-to-voxels
                 points-to-voxel-indices
                 points-track-partitions
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )

install( PROGRAMS examples/points-eigen points-to-image DESTINATION ${snark_INSTALL_BIN_DIR} )

add_executable( points-mesh points-mesh.cpp )
target_link_libraries( points-mesh ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( points-mesh PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS points-mesh RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

if( USE_CUDA )
    message( WARNING "points-join: cuda compilation broken; todo: either fix or remove" )
    #cuda_add_executable( points-join points-join.cpp points-join/points_join_cuda.cu ) # quick and dirty, simplistic for now
    add_executable( points-join points-join.cpp )
else( USE_CUDA )
    add_executable( points-join points-join.cpp )
endif( USE_CUDA )

target_link_libraries( points-join snark_math snark_math_geometry snark_point_cloud ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} tbb )
set_target_properties( points-join PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS points-join RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

if( NOT WIN32 )
    add_executable( points-grep points-grep.cpp ${points_grep_includes} ${points_grep_source} )
    target_link_libraries( points-grep snark_math_geometry snark_math ${comma_ALL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} )
    set_target_properties( points-grep PROPERTIES LINK_FLAGS_RELEASE -s )
    install( TARGETS points-grep RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
endif( NOT WIN32 )

add_subdirectory( points_calc )
