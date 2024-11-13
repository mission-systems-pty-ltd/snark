# Copyright (c) 2024 Vsevolod Vlaskine
# All rights reserved

import numpy

# todo
# - more flags: aligned, not aligned, etc
def strides( shape, kernel_shape, stride_shape, align_end=True ):
    '''
        examples
            >>> strides( ( 2000, 500, 3 ), ( 500, 200 ), ( 200, 200 ), align_end=True )
            see in strides_test.py for more examples
    '''
    assert len( kernel_shape ) == len( stride_shape ), f'expected kernel_shape and stride_shape of same size; got kernel_shape: {kernel_shape}, stride_shape: {stride_shape}'
    assert len( shape ) >= len( kernel_shape ), f'expected shape size >= kernel_shape; got shape: {shape} kernel_shape: {kernel_shape}'
    sizes = ( ( int( ( shape[i] - kernel_shape[i] ) / size + 1 ) + bool( ( shape[i] - kernel_shape[i] ) % size > 0 ), stride_shape[i] ) for i, size in enumerate( stride_shape ) )
    ranges = tuple( numpy.arange( 0, count * stride, stride, dtype=int ) for count, stride in sizes )
    values = tuple( numpy.where( r + kernel_shape[i] > shape[i], shape[i] - kernel_shape[i], r ) for i, r, in enumerate( ranges ) ) if align_end else ranges
    grid = numpy.array( numpy.meshgrid( *values ), dtype=int ).T
    return numpy.reshape( grid, ( int( numpy.prod( grid.shape ) / len( kernel_shape ) ), len( kernel_shape ) ) )

class stride_iterator:
    """
    examples
        see examples in strides_test.py
    """
    def __init__( self, input, kernel_shape, stride_shape, align_end=True ):
        self.input = input
        self.kernel_shape = kernel_shape
        self.stride_shape = stride_shape
        self.strides = strides( input.shape, kernel_shape, stride_shape, align_end=align_end )
        self._kernel_shape = numpy.array( kernel_shape, dtype=int ) # quick and dirty

    def __iter__( self ):
        for s in self.strides:
            yield s, self.input[ tuple( slice( b, b + k ) for b, k in zip( s, self._kernel_shape ) ) ]

