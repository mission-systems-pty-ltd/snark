#!/usr/bin/env python3

import setuptools
import snark.version

setuptools.setup(
        name                = 'snark',
        version             = snark.version.__version__,
        description         = 'snark python utilties',
        url                 = 'https://gitlab.com/orthographic/snark',
        license             = 'BSD 3-Clause',
        # should this be:
        # packages = setuptools.find_packages()
        packages            = [ 'snark'
                              , 'snark.imaging'
                              , 'snark.imaging.cameras'
                              , 'snark.imaging.cv'
                              , 'snark.imaging.cv.types'
                              , 'snark.math'
                              , 'snark.ros'
                              , 'snark.ros.detail'
                              , 'snark.ros2'
                              , 'snark.ros2.detail' ]
        # namespace_packages  = [ 'snark' ] # deprecated
     )
