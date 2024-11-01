#!/usr/bin/env python

# This file is part of comma, a generic and flexible library
# Copyright (c) 2011 The University of Sydney
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the University of Sydney nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
# GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
# HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
# IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''
Provides file reader for cv-cat style binary data
'''

from __future__ import print_function

import cv2, numpy, sys
from . import types

__author__ = 'j.underwood'

def safe_read(file, dtype, count):
    """Compatible with Python 2 and 3; duplicates read data for short period."""
    try:
        buf = file.read(dtype.itemsize)
        return numpy.frombuffer(buf, dtype, count)
    except ValueError:
        raise EOFError()


def make_header(rows, cols, dtype, timestamp=None):
    header = numpy.empty(1, dtype=image.header_dtype)
    header['time'] = timestamp if timestamp is not None else numpy.datetime64('now')
    header['rows'] = rows
    header['cols'] = cols
    try:
        channels = int(dtype[0])
        dtype = dtype[1:]
    except ValueError:
        channels = 1
    header['type'] = types.from_string(dtype, channels)
    return header[0]


def make_header_from(frame, timestamp=None):
    header = numpy.empty(1, dtype=image.header_dtype)
    header['time'] = timestamp if timestamp is not None else numpy.datetime64('now')
    header['rows'], header['cols'] = frame.shape[:2]
    channels = 1 if frame.ndim == 2 else frame.shape[-1]
    header['type'] = types.string2cv(frame.dtype.name, channels)
    return header[0]


class image():
    header_dtype = numpy.dtype([('time', 'datetime64[us]'),
                             ('rows', numpy.uint32),
                             ('cols', numpy.uint32),
                             ('type', numpy.uint32)])

    def __init__(self, header=None, data=None):
        self.header = header
        self.data = data

    def validate(self):
        type_string, channels = types.to_string(self.header['type'])
        if self.data.dtype.name != type_string:
            raise ValueError("cv_image dtype mismatch ({}, expected {})".format(self.data.dtype.name, type_string))
        expected_shape = (self.header['rows'], self.header['cols'], channels)
        if self.data.shape != expected_shape:
            raise ValueError("cv_image shape mismatch ({}, expected {})".format(self.data.shape, expected_shape))

    def write(self, file=sys.stdout.buffer, flush=False):
        if self.header is not None:
            self.validate()
            file.write(self.header.tobytes())
        if self.data is not None:
            file.write(self.data.tobytes())
        if flush:
            file.flush()

    @staticmethod
    def read(file, frame_dtype=None):
        """If frame_dtype is None then assume it has a header and read it"""
        header = None
        frame = None
        if frame_dtype is None:
            header = safe_read(file, image.header_dtype, 1)[0]
            type_string, channels = types.cv2string(header['type'])
            frame_dtype = numpy.dtype((numpy.dtype(type_string), (header['rows'], header['cols'], channels)))
        frame = safe_read(file, frame_dtype, 1)[0]
        return image(header, frame)


def iterator(file=sys.stdin.buffer, rows=None, cols=None, dtype=None):
    """
    read binary cv-cat data from file object in the form t,rows,cols,type,data, t,3ui,data.
    if rows, cols and dtype provided, assume there is no header in the stream.
    dtype is specified by: [num_channels]{uint8, int8, uint16, int16, int32, float32, float64}
    e.g.: iterator(sys.sys.stdin.buffer, rows=512, cols=512, dtype='3uint8')
    """
    assert (rows is None and cols is None and dtype is None) or (rows and cols and dtype), \
        "rows, cols, dtype should be all None or all not None"
    frame_dtype = None
    if rows is not None:
        try:
            channels = int(dtype[0])
            dtype = dtype[1:]
        except ValueError:
            channels = 1
        frame_dtype = numpy.dtype((numpy.dtype(dtype), (rows, cols, channels)))
    try:
        while True: yield image.read(file, frame_dtype)
    except EOFError:
        return


def parse_inputspec(input_spec):
    """input_spec=<path>[;rows=<rows>;cols=<cols>;type=<dtype>] (dtype as in cv_image.iterator)"""
    parts = input_spec.split(';')
    header = dict([tuple(part.split('=')) for part in parts if '=' in part])
    rows = int(header['rows']) if 'rows' in header else None
    cols = int(header['cols']) if 'cols' in header else None
    dtype = header['type'] if 'type' in header else None
    return parts[0], rows, cols, dtype


def zip_iterator(*inputs):
    """Iterate over image stream(s) given input specifications which can be either input_spec strings or
    tuple(file-like, input_spec or dict).
    If the input is a string, it is parameterised as:
        input_spec=<path>[;rows=<rows>;cols=<cols>;type=<dtype>] (dtype as in cv_image.iterator)
    If the input is a tuple, the first element is a file-like object (supporting .read()), and the second
    element can be either a dict specifying rows, cols and type, or an input_spec string (path should be ommitted).
    In any case, if the rows, cols and type is None, the header is read from the stream.
    """

    iterators = []
    files = []
    for input_spec in inputs:
        rows, cols, dtype = None, None, None
        if isinstance(input_spec, tuple):
            assert hasattr(input_spec[0], 'read'), "image input specified via tuple must have a file-like object"
            file = input_spec[0]
            if isinstance(input_spec[1], dict):
                rows, cols, dtype = [input_spec[1][i] for i in ('rows', 'cols', 'dtype')]
            elif isinstance(input_spec[1], str):
                _, rows, cols, dtype = parse_inputspec(";" + input_spec[1])
        else:
            path, rows, cols, dtype = parse_inputspec(input_spec)
            file = open(path, 'rb')
        files.append(file)
        iterators.append(iterator(files[-1], rows=rows, cols=cols, dtype=dtype))
    try:
        while True:
            yield tuple([next(stream_iter) for stream_iter in iterators])
    except StopIteration:
        return


def write(frame, flush=False, out_file=sys.stdout.buffer):
    """Method for backwards compatibility, use frame.write() instead."""
    frame.write(out_file, flush=False)


if __name__ == '__main__':
    for i in iterator(sys.stdin.buffer):
        print("{},{},{},{}".format(
            i.header['time'].item(),
            i.header['rows'], i.header['cols'], i.header['type']), file=sys.stderr)
        cv2.imshow('test', i.data)
        cv2.waitKey(1)
