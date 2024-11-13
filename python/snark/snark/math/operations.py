# Copyright (c) 2024 Vsevolod Vlaskine
# All rights reserved

import numpy

# todo
# - align end
# - move to math
# todo
# - align end
# - move to math
# - more flags: aligned, not aligned, etc
def strides( shape, kernel_shape, stride_shape, align_end=True ):
    '''
        examples
            >>> strides( ( 2000, 500, 3 ), ( 500, 200 ), ( 200, 200 ), align_end=True )
    '''
    assert len( kernel_shape ) == len( stride_shape ), f'expected kernel_shape and stride_shape of same size; got kernel_shape: {kernel_shape}, stride_shape: {stride_shape}'
    assert len( shape ) >= len( kernel_shape ), f'expected shape size >= kernel_shape; got shape: {shape} kernel_shape: {kernel_shape}'
    sizes = ( ( int( ( shape[i] - kernel_shape[i] ) / size + 1 ) + bool( ( shape[i] - kernel_shape[i] ) % size > 0 ), stride_shape[i] ) for i, size in enumerate( stride_shape ) )
    ranges = tuple( numpy.arange( 0, count * stride, stride, dtype=int ) for count, stride in sizes )
    values = tuple( numpy.where( r + kernel_shape[i] > shape[i], shape[i] - kernel_shape[i], r ) for i, r, in enumerate( ranges ) ) if align_end else ranges
    grid = numpy.array( numpy.meshgrid( *values ), dtype=int ).T
    return numpy.reshape( grid, ( int( numpy.prod( grid.shape ) / len( kernel_shape ) ), len( kernel_shape ) ) )

def stride_iterator( a, s ):
    """
    trivial convenience wrapper
    """
    assert len( a.shape ) >= s.shape[2] and , f'expected array dimensions greater or equal to stride shape; got: array shape: {a.shape}; stride dimensions: {s.shape[2]}'
    for t in s: yield a[t]
