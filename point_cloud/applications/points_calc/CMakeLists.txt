#if( USE_CUDA )
    #cuda_add_library( snark_point_cloud_points_join_detail_cuda points_join_detail.cu ) #cuda_compile( points_join_detail.o points_join_detail.cu OPTIONS -dc )
    #cuda_compile( snark_points_join_detail_cuda_generated points_join_cuda.cu )
#endif( USE_CUDA )
