#!/usr/bin/env python3

import comma.csv
from snark.ros2 import convert
from rosidl_runtime_py.utilities import get_message

import argparse
import os
import sys

VERBOSE = False

def say( msg: str, scriptname=os.path.basename( sys.argv[0] )):
    print( f"{scriptname}: {msg}", file=sys.stderr )

def verbose( msg: str ):
    if VERBOSE: say( msg )

type_map = {
    'int8': int, 'int16': int, 'int32': int, 'int64': int,
    'uint8': int, 'uint16': int, 'uint32': int, 'uint64': int,
    'float': float, 'double': float, 'float32': float, 'float64': float,
    'boolean': lambda x: x.lower() in ('1', 'true'),
    'octet': lambda x: bytes( [int(x)] ),
    'string': str,
    'sequence<uint8>': lambda x: bytes( int( v ) for v in x.split(",") )
}

def cast_field( msg_class, field_name, value ):
    field_type = msg_class.get_fields_and_field_types()[field_name]
    return type_map[field_type]( value )

def remove_comments( lines ):
    for line in lines:
        stripped = line.split('#', 1)[0].rstrip()
        if stripped:
            yield stripped

def process_input( is_binary ):
    output_counter = 0
    last_msg_type = ""

    for line in remove_comments( sys.stdin ):
        msg_type, msg_value = line.split("=")
        if msg_type != last_msg_type:
            output_counter = 0
            last_msg_type = msg_type

        msg_class = get_message( msg_type )
        verbose( f"msg_class={msg_class}" )
        verbose( f"value={cast_field( msg_class, 'data', msg_value )}" )
        msg = msg_class( data=cast_field( msg_class, 'data', msg_value ))
        record_t, record_ctor = convert.ros_message_to_csv_record( msg )
        verbose( f"record_t={record_t}" )
        record = record_ctor( msg )
        verbose( f"record={record}" )

        ostream = comma.csv.stream( record_t, binary=is_binary )

        if not is_binary: print( f"{msg_type}/{output_counter}", end="=" )
        ostream.write( record )
        output_counter += 1


def command_line_options():
    parser = argparse.ArgumentParser()
    parser.add_argument( '--verbose', action='store_true' )
    parser.add_argument( '--binary', action='store_true' )
    return parser.parse_args()


def main():
    global VERBOSE
    args = command_line_options()
    VERBOSE = args.verbose
    process_input( args.binary )


if __name__ == '__main__':
    main()
