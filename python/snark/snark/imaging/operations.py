import numpy

def strides( shape, kernel_shape, stride_shape, align_end=True ): # todo: more flags: aligned, not aligned, etc
    assert len( kernel_shape ) == len( stride_shape ), f'expected kernel_shape and stride_shape of same size; got kernel_shape: {kernel_shape}, stride_shape: {stride_shape}'
    assert len( shape ) >= len( kernel_shape ), f'expected shape size >= kernel_shape; got shape: {shape} kernel_shape: {kernel_shape}'
    steps=tuple( int( ( shape[i[0]] - kernel_shape[i[0]] ) / i[1] ) + ( shape[i[0]] - kernel_shape[i[0]] ) % i[1] for i in enumerate( stride_shape ) )

    # in progress...

    return steps
