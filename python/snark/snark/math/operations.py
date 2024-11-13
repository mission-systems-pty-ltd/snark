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

def stride_iterator( a, kernel_shape, stride_shape ):
    """
    ... todo
    """
    s = strides( a.shape, kernel_shape, stride_shape, align_end=True )
    k = numpy.array( kernel_shape, dtype=int )
    #for t in s: yield a[t : t + k]
    for t in s: yield t, t + k


class stride_iterator:
    """
    ... todo
    """
    def __init__( self, input, kernel_shape, stride_shape, align_end=True ):
        self.input = input
        self.kernel_shape = kernel_shape
        self.stride_shape = stride_shape
        self.strides = strides( input.shape, kernel_shape, stride_shape, align_end=align_end )
        self._kernel_shape = numpy.array( kernel_shape, dtype=int ) # quick and dirty

    def __iter__( self ):
        #for t in s: yield a[t : t + self._kernel_shape]
        for s in self.strides: yield s, s + self._kernel_shape
