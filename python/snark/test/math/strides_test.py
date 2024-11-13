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
    a = numpy.arange( 0, 24 * 3, 1 ).reshape( ( 4, 6, 3 ) )
    expected = [ [[[ 0,  1,  2], [ 3,  4,  5]], [[18, 19, 20], [21, 22, 23]], [[36, 37, 38], [39, 40, 41]]]
               , [[[ 6,  7,  8], [ 9, 10, 11]], [[24, 25, 26], [27, 28, 29]], [[42, 43, 44], [45, 46, 47]]]
               , [[[12, 13, 14], [15, 16, 17]], [[30, 31, 32], [33, 34, 35]], [[48, 49, 50], [51, 52, 53]]]
               , [[[18, 19, 20], [21, 22, 23]], [[36, 37, 38], [39, 40, 41]], [[54, 55, 56], [57, 58, 59]]]
               , [[[24, 25, 26], [27, 28, 29]], [[42, 43, 44], [45, 46, 47]], [[60, 61, 62], [63, 64, 65]]]
               , [[[30, 31, 32], [33, 34, 35]], [[48, 49, 50], [51, 52, 53]], [[66, 67, 68], [69, 70, 71]]] ]
    it = snark.math.stride_iterator( a, ( 3, 2 ), ( 2, 2 ) )
    assert len( it.strides ) == len( expected )
    i = 0
    for stride, array in it:
        assert numpy.all( array == expected[i] )
        i += 1
    # todo! more tests
