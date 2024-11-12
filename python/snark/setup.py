#!/usr/bin/env python3

from distutils.core import setup

setup(
        name                = 'snark',
        version             = open('snark/version.py').readlines()[-1].strip().split()[-1].strip('\"'),
        description         = 'snark python utilties',
        url                 = 'https://gitlab.com/orthographic/snark',
        license             = 'BSD 3-Clause',
        packages            = [ 'snark'
                              , 'snark.imaging'
                              , 'snark.imaging.cameras'
                              , 'snark.imaging.cv'
                              , 'snark.imaging.cv.types'
                              , 'snark.math'
                              , 'snark.ros'
                              , 'snark.ros2' ]
        # namespace_packages  = [ 'snark' ] # deprecated
     )
