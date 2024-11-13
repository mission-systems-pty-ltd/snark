import pytest, numpy
import snark.math

def test_strides():
    assert numpy.all( snark.math.strides( ( 800, 400, 3 ), ( 800, 400 ), ( 800, 400 ), align_end=True ) == [[0, 0]] )
    assert numpy.all( snark.math.strides( ( 800, 400, 3 ), ( 800, 400 ), ( 800, 400 ), align_end=False ) == [[0, 0]] )
    assert numpy.all( snark.math.strides( ( 801, 400, 3 ), ( 800, 400 ), ( 800, 400 ), align_end=True ) == [[0, 0], [1, 0]] )
    assert numpy.all( snark.math.strides( ( 801, 400, 3 ), ( 800, 400 ), ( 800, 400 ), align_end=False ) == [[0, 0], [800, 0]] )
    assert numpy.all( snark.math.strides( ( 800, 401, 3 ), ( 800, 400 ), ( 800, 400 ), align_end=True ) == [[0, 0], [0, 1]] )
    assert numpy.all( snark.math.strides( ( 800, 401, 3 ), ( 800, 400 ), ( 800, 400 ), align_end=False ) == [[0, 0], [0, 400]] )
    assert numpy.all( snark.math.strides( ( 801, 401, 3 ), ( 800, 400 ), ( 800, 400 ), align_end=True ) == [[0, 0], [0, 1], [1, 0], [1, 1]] )
    assert numpy.all( snark.math.strides( ( 801, 401, 3 ), ( 800, 400 ), ( 800, 400 ), align_end=False ) == [[0, 0], [0, 400], [800, 0], [800, 400]] )
    assert numpy.all( snark.math.strides( ( 1000, 500, 3 ), ( 800, 200 ), ( 400, 400 ), align_end=True ) == [[0, 0], [0, 300],[200, 0], [200, 300]] )
    assert numpy.all( snark.math.strides( ( 1000, 500, 3 ), ( 800, 200 ), ( 400, 400 ), align_end=False ) == [[0, 0], [0, 400],[400, 0], [400, 400]] )
    # todo! more tests

def test_stride_iterator():
    a = numpy.arange( 0, 24, 1 ).reshape( ( 4, 6 ) )
    for i in snark.math.stride_iterator( a, ( 3, 4 ), ( 2, 2 ) ):
        print( i )

    # s = snark.math.strides( ( 4, 6, 2 ), ( 3, 4 ), ( 2, 2 ) )
    # a = numpy.arange( 0, 48, 1 ).reshape( ( 4, 6, 2 ) )
    # for i in snark.math.stride_iterator( a, s ):
    #     print( i )