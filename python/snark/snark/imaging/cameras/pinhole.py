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

    def field_of_view_2tan( self ): # todo: add other modalities
        assert self.focal_length > 0, f'expected positive focal length, got {self.focal_length}'
        dimensions = ( self.sensor_size if self.sensor_size else self.image_size ).as_numpy( dtype=float )
        assert dimensions[0] > 0 and dimensions[1] > 0, f'expected positive dimensions, got {dimension}'
        return dimensions / self.focal_length

    def field_of_view( self ): return 2. * numpy.arctan( self.field_of_view_2tan() / 2. )

    def get_principal_point( self ): return self.principal_point if self.principal_point else Pinhole.Point2d( x=0.5 * self.image_size.x, y=0.5 * self.image_size.y )

    def set_principal_point( self ): self.principal_point = self.get_principal_point()
