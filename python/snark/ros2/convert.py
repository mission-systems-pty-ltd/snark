#!/usr/bin/env python3

import numpy
import rclpy
import comma
import datetime
import re
import sys

try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    msg = """
cannot import rclpy_message_converter module; usually you can install it as
    sudo apt-get install ros-humble-rospy-message-converter
(use your ROS distro name in place of humble). If the module is not available
in your package manager, build and install the module manually.

you can install it from source as:
> git clone https://github.com/DFKI-NI/rospy_message_converter.git
> cd rospy_message_converter
> git checkout humble # or any other branch
> python setup.bash build
> python setup.bash install
"""
    raise ImportError(msg)

def ros_message_to_csv_record(message, lengths={}, ignore_variable_size_arrays=True):
    record_t, record_ctor = _ros_message_to_csv_record(message, lengths=lengths, ignore_variable_size_arrays=ignore_variable_size_arrays, prefix='')
    for k, v in lengths.items():
        try:
            pos = record_t.fields.index(k)
            if record_t.types[pos][0] != 'S':
                raise RuntimeError("length %d specified for field '%s' that is not a string" % (v, k))
        except ValueError:
            raise RuntimeError("length %d specified for unknown field '%s'" % (v, k))
    return record_t, record_ctor

def from_csv_supported_types(v):
    if type(v) != numpy.datetime64:
        return v
    microseconds = numpy.int64(v)
    return rclpy.time.Time(seconds=microseconds // 1000000, nanoseconds=(microseconds % 1000000) * 1000)

def is_binary_type(field_type):
    from rosidl_parser.definition import AbstractNestedType, AbstractString
    if isinstance(field_type, AbstractNestedType) or isinstance(field_type, AbstractString):
        return False
    #return field_type.type in ('octet', 'uint8')
    return field_type.__name__ in ('octet', 'uint8')

def _ros_message_to_csv_record_impl(message, lengths={}, ignore_variable_size_arrays=True, prefix=''):
    from rosidl_runtime_py import message_to_ordereddict
    from rosidl_parser.definition import AbstractSequence, AbstractString, BasicType

    full_path = lambda name: prefix and prefix + "/" + name or name

    message_dict = message_to_ordereddict(message)
    fields = []
    types = []
    ctors = []

    for field_name, field_value in message_dict.items():
        field_type = type(field_value)
        if is_binary_type(field_type):
            ctor = lambda msg, field_name=field_name: getattr(msg, field_name)
            current_path = full_path(field_name)
            try:
                l = lengths[current_path]
            except KeyError:
                l = len(ctor(message))
            element_t = "S%d" % l
        elif isinstance(field_type, (BasicType, AbstractString)):
            ctor = lambda msg, field_name=field_name: getattr(msg, field_name)
            if field_type in (AbstractString,):
                current_path = full_path(field_name)
                try:
                    l = lengths[current_path]
                except KeyError:
                    l = len(ctor(message))
                element_t = "S%d" % l
            else:
                element_t = field_type.type
        elif isinstance(field_type, rclpy.time.Time):
            def ctor(msg, field_name=field_name):
                ts = getattr(msg, field_name)
                return numpy.datetime64(datetime.datetime.utcfromtimestamp(ts.seconds + 1.0e-9 * ts.nanoseconds))
            element_t = 'datetime64[us]'
        elif isinstance(field_type, rclpy.duration.Duration):
            def ctor(msg, field_name=field_name):
                ts = getattr(msg, field_name)
                return numpy.timedelta64(ts.seconds, 's') + numpy.timedelta64(ts.nanoseconds, 'ns')
            element_t = 'timedelta64[us]'
        elif isinstance(field_type, AbstractSequence):
            ctor = lambda msg, field_name=field_name: getattr(msg, field_name)
            list_brackets = re.compile(r'\[[^\]]*\]')
            m = list_brackets.search(field_type.type)
            size_string = m.group()[1:-1]
            size = 0 if size_string == '' else int(size_string)
            if size == 0 and ignore_variable_size_arrays:
                continue
            element_t = (field_type.type[:m.start()], (size,))
        else:
            element_t, element_ctor = _ros_message_to_csv_record_impl(getattr(message, field_name), lengths=lengths, ignore_variable_size_arrays=ignore_variable_size_arrays, prefix=full_path(field_name))
            ctor = lambda msg, field_name=field_name, element_ctor=element_ctor: element_ctor(getattr(msg, field_name))

        fields.append(field_name)
        ctors.append(ctor)
        types.append(element_t)

    new_t = comma.csv.struct(','.join(fields), *types)
    return new_t, lambda msg, new_t=new_t: tuple([c(msg) for c in ctors])

def _ros_message_to_csv_record(message, lengths={}, ignore_variable_size_arrays=True, prefix=''):
    t, f = _ros_message_to_csv_record_impl(message, lengths, ignore_variable_size_arrays, prefix)
    return t, lambda msg: numpy.array([f(msg)], dtype=t)
