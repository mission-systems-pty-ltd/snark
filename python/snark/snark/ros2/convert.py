#!/usr/bin/env python3

import numpy
import rclpy
import comma
import datetime
import re

def ros_message_to_csv_record( message, lengths={}, ignore_variable_size_arrays=True ):
    """
    Takes a ROS message and returns a comma.csv.struct (Python type) representing this message
    and a lambda function converting the message into an instance of the new comma.csv.struct
    Optional second argument allows to specify explicitly the length of variable-length values,
    such as strings. By default, take the lengths from the message itself.
"""
    record_t, record_ctor = _ros_message_to_csv_record( message, lengths=lengths,
                                                        ignore_variable_size_arrays=ignore_variable_size_arrays,
                                                        prefix='' )
    for k, v in lengths.items():
        try:
            pos = record_t.fields.index(k)
            if record_t.types[pos][0] != 'S':
                raise RuntimeError("length %d specified for field '%s' that is not a string" % (v, k))
        except ValueError:
            raise RuntimeError("length %d specified for unknown field '%s'" % (v, k))
    return record_t, record_ctor

def from_csv_supported_types(v):
    import builtin_interfaces
    if type(v) is numpy.bytes_:
        return v.decode('utf-8')
    elif type(v) is numpy.datetime64:
        microseconds = int( numpy.int64(v) )
        return builtin_interfaces.msg.Time( sec=microseconds // 1000000, nanosec=( microseconds % 1000000 ) * 1000 )
    elif numpy.isscalar( v ):
        return v.item()
    else:
        return v


# --- type predicates -----

def is_binary_type( field_type: str ) -> bool:
    return ( field_type.startswith( ("uint8[", "byte[", "octet[") ) or
             field_type in ("sequence<uint8>", "sequence<byte>", "sequence<octet>") )

# types taken from ROS 1 message_converter.ros_primitive_types
# https://github.com/DFKI-NI/rospy_message_converter/blob/master/src/rospy_message_converter/message_converter.py#L105
# replacing 'char' with 'octet'
def is_primitive_type( field_type: str ) -> bool:
    return field_type in ['bool', 'byte', 'octet', 'int8', 'uint8', 'int16', 'uint16', 'int32',
                          'uint32', 'int64', 'uint64', 'float32', 'float64', 'float', 'double', 'string']

def is_time_type( field_type: str ) -> bool:
    return field_type == "builtin_interfaces/Time"

def is_duration_type( field_type: str ) -> bool:
    return field_type == "builtin_interfaces/Duration"

def is_array_type( field_type: str ) -> bool:
    return field_type.find('[') >= 0 or field_type.startswith("sequence")

def is_variable_array( field_type: str ) -> bool:
    return field_type.find('[]') >= 0 or field_type.startswith("sequence")

def is_fixed_array( field_type: str ) -> bool:
    left_bracket = field_type.find('[')
    return left_bracket > 0 and field_type[ left_bracket + 1 ] != ']'

# -------------------------

def message_fields( message ):
    "take an instantiated message and return field name and type as list of tuples"
    return message.get_fields_and_field_types().items()

def _ros_message_to_csv_record_impl( message, lengths={}, ignore_variable_size_arrays=True, prefix='' ):
    "Private implementation of ros_message_to_csv_record. Called recursively."

    full_path = lambda name: prefix and prefix + "/" + name or name

    fields = []
    types = []
    ctors = []

    for field_name, field_type_str in message_fields( message ):
        field_value = getattr( message, field_name )
        if is_binary_type( field_type_str ):
            ctor = lambda msg, field_name=field_name: getattr( msg, field_name )
            current_path = full_path(field_name)
            try:
                length = lengths[current_path]
            except KeyError:
                length = len(ctor(message))
            element_t = "S%d" % length
        elif is_primitive_type( field_type_str ):
            ctor = lambda msg, field_name=field_name: getattr( msg, field_name )
            if field_type_str in ['string']:
                ctor = lambda msg, field_name=field_name: getattr(msg, field_name)
                current_path = full_path(field_name)
                try:
                    length = lengths[current_path]
                except KeyError:
                    length = len(ctor(message))
                element_t = "S%d" % length
            else:
                element_t = field_type_str
        elif is_time_type( field_type_str ):
            def ctor(msg, field_name=field_name):
                ts = getattr(msg, field_name)
                return numpy.datetime64(datetime.datetime.utcfromtimestamp(ts.seconds + 1.0e-9 * ts.nanoseconds))
            element_t = 'datetime64[us]'
        elif is_duration_type( field_type_str ):
            def ctor(msg, field_name=field_name):
                ts = getattr(msg, field_name)
                return numpy.timedelta64(ts.seconds, 's') + numpy.timedelta64(ts.nanoseconds, 'ns')
            element_t = 'timedelta64[us]'
        elif is_array_type( field_type_str ):
            ctor = lambda msg, field_name=field_name: getattr(msg, field_name)
            if is_variable_array( field_type_str ):
                size = 0
            else:
                list_brackets = re.compile( r'\[[^\]]*\]' )
                m = list_brackets.search( field_type_str )
                size_string = m.group()[1:-1]
                size = 0 if size_string == '' else int(size_string)
            if size == 0 and ignore_variable_size_arrays: continue
            element_t = (field_type_str[:m.start()], (size,))
        else:
            element_t, element_ctor = _ros_message_to_csv_record_impl( getattr( message, field_name ),
                                                                       lengths=lengths,
                                                                       ignore_variable_size_arrays=ignore_variable_size_arrays,
                                                                       prefix=full_path( field_name ))
            ctor = lambda msg, field_name=field_name, element_ctor=element_ctor: element_ctor(getattr(msg, field_name))

        fields.append( field_name )
        ctors.append( ctor )
        types.append( element_t )

    new_t = comma.csv.struct(','.join(fields), *types)
    return new_t, lambda msg, new_t=new_t: tuple([c(msg) for c in ctors])

def _ros_message_to_csv_record(message, lengths={}, ignore_variable_size_arrays=True, prefix=''):
    t, f = _ros_message_to_csv_record_impl(message, lengths, ignore_variable_size_arrays, prefix)
    return t, lambda msg: numpy.array([f(msg)], dtype=t)
