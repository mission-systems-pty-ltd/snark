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