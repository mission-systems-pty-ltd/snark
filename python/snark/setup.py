#!/usr/bin/env python

from distutils.core import setup

setup(
        name                = 'snark',
        version             = open('snark/version.py').readlines()[-1].strip().split()[-1].strip('\"'),
        description         = 'snark python utilties',
        url                 = 'https://github.com/acfr/snark',
        license             = 'BSD 3-Clause',
        packages            = [ 'snark', 'snark.imaging', 'snark.ros', 'snark.ros.detail', 'snark.ros2', 'snark.ros2.detail'],
     )
