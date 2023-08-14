# Working with libximc

## With pip3

``` pip3 install libximc ```

## Manual

If pip install does not work:

This script requires the following libraries from Standa
- libximc.dll
- bindy.dll
- xiwrapper.dll

1. Download the latest libximc library from https://files.xisupport.com/Software.en.html
2. Extract and save to a suitable location. By default `standa-pan-tilt-control` will search for the ximc directory at … although users may specify alternate locations as a command line argument
3. Depending on what architecture you are using
    1. If you are using windows, no more steps are required. The python wrapper points to the required packages
    2. If using a Unix-based system, you must install the packages for the required architecture
        1. Packages are found at `ximc/deb`. Install libximc7_x.x.x and libximc7-dev_x.x.x for the required architecture **strictly in the specified order**
        2. Now you must specify the correct path to the installed packages using `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:usr/lib` (or whatever path to where the packages were installed)
5. If everything has been done correctly, `standa-pan-tilt-control` will now import the module `pyximc.py` from the `ximc` directory. `pyximc.py` is a python wrapper which will load the external packages needed to use `libximc`

For further help, see the examples included in the `ximc` directory, or read the user manual available at https://doc.xisupport.com/en/ 

# Specs
## Stepper and DC Motor Controller
https://www.standa.lt/products/catalog/motorised_positioners?item=525

## Pan Tilt Table
https://www.standa.lt/products/catalog/custom_engineering?item=546&prod=TBA

### Conversion
1 step = 0.6 arcmin

# Using the program

## Usage
The primary interface with this script is through CLI. The script is run as
```bash
standa-pan-tilt-control [options]
```
## Modes
The PTU has four possible modes; 'HOME', 'RUN', 'RUN_WIHTOUT_HOMING', and 'MOVE'.
 * HOME: moves to a preconfigured home
 * RUN: moves to home, then begins a scanning routine, outputting <timestamp>,<azimuth>,<elevation> to stdout as binary. The scanning routine may either be specifed using a config file (a path to the config file may be specified as a command line argument), else default values.
 * RUN_WITHOUT_HOMING: same as above, but does not HOME first. **NOT RECOMMENDED!!**
 * MOVE: Allows direct user access through the console. Accepts user input in binary <azimuth>,<elevation>. All values are relative to the current position (ie, device will move by offset <azimuth>,<elevation> from current position)

### Configuration Files

The program expects a JSON dict as a config file. It will accept a path to a config file as a command line argument. The config file has the following parameters
* angular_resolution: the desired resolution of position measurements, written to stdout. (RADIANS)
* output_rate: the rate at which position measurements are taken. Angular resolution and output rate determine speed of movement according to angular_resolution*output_rate. Note that the maximum possible speed is 100,000 steps/second (100,000 * 0.01 * pi/180 ~= 17.45 rad/s) (HERTZ)
* azimuth: a dictionary containg the maximum ("max") and minimum ("min") values for azimuth
* elevation: a dictionary containg the maximum ("max") and minimum ("min") values for elevation

```
{
    "angular_resolution": 0.000174533,
    "output_rate": 100,
    "azimuth": {"min": 1.57, "max": -1.57},
    "elevation": {"min": -0.17, "max": 1.57}
}
```
### Examples 
#### Keyboard Control
The pan tilt unit can be controlled with the keyboard arrows. An example of the program call for keyboard control, outputting the angular position measurement to stdout as csv is

```
io-console | control-from-console pantilt | csv-to-bin 2f --flush | standa-pan-tilt-control --speed 0.2 |csv-from-bin t,2f
```

#### Input Stream Control
The pan tilt unit can be controlled by an input stream. The program expects binary values in the format AZIMUTH,ELEVATION. The program interprets values as offset movements. For this reason it is not recommended to chain inputs. An example of a program call, outputting angular position measurement as binary is 
```
csv-to-bin 2f --flush | standa-pan-tilt-control
```

#### Home Device
The pan tilt unit can be sent to home as follows
```
stand-pan-tilt-control --mode HOME --speed 0.2
```

#### Configured Scan Routine
The pan tilt unit can be run according to a set routine specified as a config file. The device can be homed before running the routine, or can begin routine immediately. WARNING!!! If the device is not homed before running command, make sure that the routine will not cause a physical collision. It is recommended to always home the device first.

```
standa-pan-tilt-control --mode RUN --logging ERROR --config <PATH TO CONFIG> | csv-from-bin t,2f
```

```
standa-pan-tilt-control --mode RUN_NO_HOME --logging ERROR --config <PATH TO CONFIG> | csv-from-bin t,2f
```
# Hardware Setup
The Standa Controller has two outputs; Axis 1 and Axis 2. Axis 1 should be connected to the azimuth motor. Axis 2 should be connected to the elevation motor. The software is written with this assumption, and rewiring without changing the code will result in the controller being incorrectly configured.

# Warning!!
This device does not have physical limit switches. If the device is zero'd/moved out of bounds, it might result in a physical collision which can damage the motors. Care should be taken when reconfiguring/using this device. It is recommended to first check the coordinates of the pan tilt unit are as you expect by manually controlling the device with keyboard control.