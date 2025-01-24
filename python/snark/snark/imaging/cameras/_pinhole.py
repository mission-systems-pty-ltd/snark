# Copyright (c) 2024 Vsevolod Vlaskine

import numpy, typing
from dataclasses import dataclass, field
from dataclasses_json import dataclass_json

@dataclass_json
@dataclass
class Pinhole:
    @dataclass_json
    @dataclass
    class Point2d: # todo: move someplace generic
        x: float = 0
        y: float = 0
    
    @dataclass_json
    @dataclass
    class Point3d: # todo: move someplace generic
        x: float = 0
        y: float = 0
        z: float = 0

    @dataclass_json
    @dataclass
    class Point2i: # todo: move someplace generic
        x: float = 0
        y: float = 0

    @dataclass_json
    @dataclass
    class Distortion:

        @dataclass_json
        @dataclass
        class Radial:
            k1: float = 0
            k2: float = 0
            k3: float = 0

        @dataclass_json
        @dataclass
        class Tangential:
            p1: float = 0
            p2: float = 0

        radial: Radial = field( default_factory=lambda: Pinhole.Distortion.Radial() )
        tangential: Tangential = field( default_factory=lambda: Pinhole.Distortion.Tangential() )
        map: str = '' # filename with the distortion map

    sensor_size: typing.Optional[Point2d] = field( default_factory=lambda: None )
    image_size: Point2i = field( default_factory=lambda: Pinhole.Point2i() )
    principal_point: typing.Optional[Point2d] = field( default_factory=lambda: None )
    focal_length: float = 0
    distortion: typing.Optional[Distortion] = field( default_factory=lambda: None )

    def empty(): return image_size.x == 0 or image_size.y == 0 # quick and dirty

    def field_of_view_2tan( self ): # todo: add other modalities
        assert self.focal_length > 0, f'expected positive focal length, got {self.focal_length}'
        dimensions = ( self.sensor_size if self.sensor_size else self.image_size ).as_numpy( dtype=float )
        assert dimensions[0] > 0 and dimensions[1] > 0, f'expected positive dimensions, got {dimension}'
        return dimensions / self.focal_length

    def field_of_view( self ): return 2. * numpy.arctan( self.field_of_view_2tan() / 2. )

    def get_principal_point( self ): return self.principal_point if self.principal_point else Pinhole.Point2d( x=0.5 * self.image_size.x, y=0.5 * self.image_size.y )

    def set_principal_point( self ): self.principal_point = self.get_principal_point()

    def camera_matrix( self ):
        
        fx = self.focal_length * ( 1. if self.sensor_size is None else self.image_size.x / self.sensor_size.x )
        fy = self.focal_length * ( 1. if self.sensor_size is None else self.image_size.y / self.sensor_size.y )
        pp = self.get_principal_point()
        cx = pp.x
        cy = pp.y
        return numpy.array( [ [ fx,  0, cx ]
                            , [  0, fy, cy ]
                            , [  0,  0,  1 ] ] )
    
    def distortion_coefficients( self ):
        return None if self.distortion is None else numpy.array( [ self.distortion.radial.k1, self.distortion.radial.k2, self.distortion.tangential.p1, self.distortion.tangential.p2, self.distortion.radial.k3 ] )

    # convert pixel coordinates to 3D point projected in camera frame with z axis pointing out of the camera and xy origin at image centroid
    def to_cartesian( self, pixel ): # geometry.Point2i
        assert pixel[0] >= 0 and pixel[1] >= 0, f'expected non-negative pixel coordinates, got {pixel}'
        assert pixel[0] < self.image_size.x and pixel[1] < self.image_size.y, f'pixel coordinates {pixel} out of bounds'
        fov = self.field_of_view()
        return Point3d( x=self.focal_length*numpy.tan(pixel[0]/self.image_size.x*fov[0] - fov[0]/2), y=self.focal_length*numpy.tan(pixel[1]/self.image_size.y*fov[1] - fov[1]/2), z=self.focal_length )