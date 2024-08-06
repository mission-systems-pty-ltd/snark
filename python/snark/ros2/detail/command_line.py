#!/usr/bin/env python3

def lengths_of_strings( fields, formats, delimiter=',' ):
    """
given delimiter-separated strings of field names and formats, returns a dictionary of { 'field': string_length } entries for all the string fields
if the arguments are not strings, assume that they have been already split by the delimiter

example:
    lengths_of_strings( 'foo,bar,baz/blah', 's[2],d,s[3]' )
returns:
    { 'foo': 2, 'baz/blah': 3 }
"""
    format_types = type(formats) == str and formats.split( delimiter ) or formats
    field_names = type(fields) == str and fields.split( delimiter ) or fields
    if len( format_types ) != len( field_names ):
        raise ValueError( "fields '%s' and formats '%s' must have the same number of elements" % ( fields, formats ) )
    lengths = {}
    import re
    regex = r'^s\[(\d+)\]$'
    for k, v in zip( field_names, format_types ):
        m = re.match( regex, v )
        if m:
            lengths[k] = int(m.group(1))
    return lengths

def fields_of_record( record_type, index_output_fields=False ):
    if index_output_fields:
        import numpy
        output_fields = []
        for n in record_type.flat_dtype.names:
            shape = record_type.flat_dtype[n].shape
            if shape:
                tmp = numpy.zeros( shape )
                it = numpy.nditer( tmp, flags=['multi_index'] )
                while not it.finished:
                    output_fields.append( n + ''.join( [ "[%s]" % i for i in it.multi_index ] ) )
                    it.iternext()
            else:
                output_fields.append( n )
        return output_fields
    else:
        return record_type.fields
