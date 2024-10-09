#!/usr/bin/env python3 

import numpy
import rclpy
import rclpy.time
from builtin_interfaces.msg import Time as builtin_time
import comma
import datetime
import re
import sys
import std_msgs.msg
try:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py import message_to_ordereddict
    from rosidl_parser.definition import AbstractSequence, AbstractNestedType, AbstractString, BasicType
    import rclpy_message_converter.message_converter as mc
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
    if isinstance( v, numpy.bytes_ ): return str( v )
    if type(v) != numpy.datetime64:
        return v
    microseconds = numpy.int64(v)
    return rclpy.time.Time(seconds=microseconds // 1000000, nanoseconds=(microseconds % 1000000) * 1000)

def is_binary_type(field_type):
    # from rosidl_parser.definition import AbstractNestedType, AbstractString
    if isinstance(field_type, AbstractNestedType) or isinstance(field_type, AbstractString):
        return False
    #return field_type.type in ('octet', 'uint8')
    return field_type.__name__ in ('octet', 'uint8')

def _ros_message_to_csv_record_impl(message, lengths={}, ignore_variable_size_arrays=True, prefix=''):

    full_path = lambda name: prefix and prefix + "/" + name or name
    # If there are no __slots__ in the message, it is a builtin type, we should return
    if not hasattr(message, '__slots__'):
        if isinstance(message, (int, float, bool, str)):
            return comma.csv.struct('value', message.__class__.__name__), lambda msg: (msg,)
        else:
            return comma.csv.struct('value', 'string'), lambda msg: (str(msg),)
            
    message_dict = message_to_ordereddict(message)
    fields = []
    types = []
    ctors = []

    if isinstance(message, builtin_time):
        field_name = "sec"
        def ctor( msg, field_name=field_name ):
            ts = getattr( msg, field_name )
            if (isinstance(ts, int)):
                return numpy.datetime64( datetime.datetime.utcfromtimestamp( ts ) )
            return numpy.datetime64( datetime.datetime.utcfromtimestamp( ts.sec + 1.0e-9 * ts.nsecs ) )
        element_t = 'datetime64[us]'
        fields.append( field_name )
        ctors.append( ctor )
        types.append( element_t )
        new_t = comma.csv.struct( ','.join( fields ), *types )
        return new_t, lambda msg, new_t=new_t: tuple( [ c(msg) for c in ctors ] )

    for field_name, field_value in message_dict.items():
        field_type = type(field_value)
        
        # if is_binary_type(field_value):
        if isinstance(field_value, (AbstractNestedType, AbstractString)):
            print("     ===>> TODO: field_type is binary", file=sys.stderr) # see ros1 convert.oy for example
        elif field_type == 'time':
            print("     ===>> asdf2, field_type is rclpy.time.Time", file=sys.stderr) # see ros1 convert.oy for example
        elif isinstance(field_value, (BasicType, AbstractString)):
            print("     ===>> field_type is BasicType or AbstractString", file=sys.stderr) # see ros1 convert.oy for example
        elif isinstance(field_value, (tuple)): 
            print("     ===>> field_type is tuple", file=sys.stderr) # see ros1 convert.oy for example
            ctor = lambda msg, field_name=field_name: getattr( msg, field_name )
            list_brackets = re.compile( r'\[[^\]]*\]' )
            m = list_brackets.search( field_type )
            size_string = m.group()[1:-1]
            size = 0 if size_string == '' else int( size_string )
            if size == 0 and ignore_variable_size_arrays: continue
            element_t = ( field_type[:m.start()], ( size, ) )
        elif isinstance(field_value, str):
            ctor = lambda msg, field_name=field_name: getattr( msg, field_name )
            current_path = full_path( field_name )
            try: l = lengths[ current_path ]
            except KeyError: l = len( ctor( message ) )
            element_t = "S%d" % l

        elif isinstance(field_value, (int, float, bool, str)):
            ctor = lambda msg, field_name=field_name: getattr( msg, field_name )

            if field_type == 'string' or field_type == 'str':
                current_path = full_path( field_name )
                try: l = lengths[ current_path ]
                except KeyError: l = len( ctor( message ) )
                element_t = "S%d" % l
            else:
                element_t = field_type
        else:
            element_t, element_ctor = _ros_message_to_csv_record_impl(getattr(message, field_name), lengths=lengths, ignore_variable_size_arrays=ignore_variable_size_arrays, prefix=full_path(field_name))
            ctor = lambda msg, field_name=field_name, element_ctor=element_ctor: element_ctor( getattr( msg, field_name ) )

        fields.append(field_name)
        ctors.append(ctor)
        types.append(element_t)

    new_t = comma.csv.struct(','.join(fields), *types)
    return new_t, lambda msg, new_t=new_t: tuple([c(msg) for c in ctors])

def _ros_message_to_csv_record(message, lengths={}, ignore_variable_size_arrays=True, prefix=''):
    t, f = _ros_message_to_csv_record_impl(message, lengths, ignore_variable_size_arrays, prefix)
    return t, lambda msg: numpy.array([f(msg)], dtype=t)
