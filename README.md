# bimvee - Batch Import, Manipulation, Visualisation, and Export of Events etc.

<img src="https://raw.githubusercontent.com/event-driven-robotics/bimvee/master/images/events.png" width="300"/> <img src="https://raw.githubusercontent.com/event-driven-robotics/bimvee/master/images/frames.png" width="300"/>
<img src="https://raw.githubusercontent.com/event-driven-robotics/bimvee/master/images/imu.png" width="300"/>
<img src="https://raw.githubusercontent.com/event-driven-robotics/bimvee/master/images/pose.png" width="300"/>
<img src="https://raw.githubusercontent.com/event-driven-robotics/bimvee/master/images/dvslastts.png" width="300"/>
<img src="https://raw.githubusercontent.com/event-driven-robotics/bimvee/master/images/eventrate.png" width="300"/>

# Quickstart

## Installation

There is a pip installer:

    pip install bimvee

Important! If you clone this repo, use --recurse-submodules option, as this repo uses 'importRosbag' library as a submodule.

## Usage

Look at [examples.py](https://github.com/event-driven-robotics/bimvee/blob/master/examples/examples.py) for examples of how to use the functionality in this library.

Want to play back your timestamped multi-channel data? Consider using https://github.com/event-driven-robotics/mustard

# Introduction

Working with timestamped address-event data from event cameras (dvs), and
possibly other neuromorphic sensors alongside other timestamped data
that we need for our experiments, including but not limited to:
- frame-based camera images
- IMU
- 6-DOF poses
- derived datatypes, such as optical (flow) events, or labelled dvs events (dvsL) etc
- Camera calibration info is also imported from e.g. ros (cam)

File formats supported include:
- IIT YARP .log - ATIS Gen1 and IMU
- rpg_dvs_ros - DVS/DAVIS .bag
- Third-party datasets recorded by using the above rosbag importer (e.g. Penn MvSEC, UMD EvIMO, Intel Realsense etc)
- Vicon - as dumped by yarpDumper
- Samsung (SEC) Gen3 VGA .bin
- Pull requests welcome for importers or exporters of other file formats.

# Contents of library

## Import functions:

The aim is to bring the different formats into as common a format as possible.
Parameters: at least the param "filePathOrName" (otherwise working from current directory)
Returns a dict containing:

    {'info': {<filePathOrName, any other info derivable from file headers>},

    'data': {

         channel0: {}
         channel1: {}
         ...
         }}

The 'data' branch contains a dict for each channel. A 'channel' is an arbitrary
grouping of datasets. It might be that there is one channel for each sensor,
so for example a file might contain 'left' and 'right'
camera channels, and each of these channels might contain dvs events alongside
other data types like frames.
Each channel is a dict containing one dict for each type of data.
Data types may include:
- dvs (Timestamped (ts) 2D address-events (x, y) with polarity (pol), from an event camera)
- frame
- imu
- flow
- pose
- etc

dvs data type, for example, then contains:

- "pol": numpy array of bool
- "x": numpy array of np.uint16
- "y": numpy array of np.uint16
- "ts": numpy array of np.float64

timestamps are always converted to seconds;
(raw formats are, however, e.g. int with unit increments of 80 ns for ATIS,
int with unit increments of 1 us for DAVIS, etc)

To the extent possible, dvs polarity is imported so that 1/True = ON/increase-in-light and
0/False = OFF/decrease-in-light. Be aware that individual datasets may contain the opposite convention.

Multiple files imported simultaneously appear in a list of dicts;
lists and dicts are referred to jointly as containers,
and the manipulation, visualistation and export functions which follow
tend toward accepting containers with an arbitrarily deep hierarchy.

## Visualisation functions

There is a set of general functions for common visualisations of imported datasets, using matplotlib or seaborn.

- plotDvsContrast
- plotDvsLastTs
- plotSpikeogram
- plotEventRate
- plotFrame
- plotImu
- plotPose
- plotCorrelogram
- plotFlow

These functions have several kwargs to modify their behaviour, and they support a 'callback' kwarg so you can pass in a function to do post-modification of the plots.

There are two different visualisation concepts. In the 'continuous' concept, a single plot shows all timestamped data for a given container. This might be limited to a certain time range, as defined by kwargs minTime and maxTime. Examples include:
- plotEventRate
- plotImu
- plotPose
- plotSpikeogram

In the 'snapshot' concept, a representation is generated for a chosen moment in time. In the case of frames this might be the nearest frame to the chosen time. In the case of dvs events this might be an image composed of events recruited from around that moment in time, for which there is a concept of the time window. In the case of poses this might be a projected view of the pose at the given time, where the pose might be linearly interpolated between the two nearest timestamped poses. Examples include:
- plotDvsContrastSingle
- plotDvsLastTs (in this case, the visualisation is based on all data up to the chosen time)

In the case of the snapshot views, there are general functions which when passed a data container will choose a set of times distributed throughout the time range of that data and generate a snapshot view for each of these moments. Examples include:
- plotDvsContrast
- plotFrame

'visualiser.py' defines a set of classes, one for each of a selection of data types, which generate snapshot views. These are output as numpy arrays, to be rendered by an external application.

info.py includes various functions to give quick text info about the contents of the containers that result from imports.

## Manipulation functions

There are some functions for standard manipulations of data:

timestamps.py contains timestamp manipulations
including jointly zeroing timestamps across multiple files, channels and datatypes.
split.py includes various common ways by which datasets need to be split, e.g. splitByPolarity

## Export functions

exportIitYarp - exports to IIT's EDPR YARP format. Alongside data.log and
info.log files, it exports an xml which specifies to yarpmanager how to
visualise the resulting data.

# Dependencies:

This library uses importRosbag library to import rosbag data without needing a ros installation.
This is included as a submodule.

Beyond the python standard library, the main dependencies are:

- numpy
- tqdm (for progress bars during import and export functions)

For the 'plot' family of visualisation functions:

- matplotlib
- mpl_toolkits (only for certain 3d visualisations)
- seaborn

The "visualiser", however, generates graphics as numpy arrays
without reference to matplotlib, for rendering by an external application.

plotDvsLastTs uses rankdata from scipy; however if it's not installed,
it defaults to a local definition; scipy is therefore an optional dependency.

undistortEvents function in events.py uses cv2 (openCv).

import/export Hdf5 functions use:

- hickle

# Type definitions

bimvee doesn't use classes for datatypes. Consequently, the code doesn't have a central place to refer to for the definition of datatypes. The types are intended to be used loosely, with minimal features which can be extended by adding optional fields. There is an optional container class which gives some functions for easier data manipulation.

There are some datatypes which are simply dicts which act as containers to group information, for example the 'cam' type. However most of the functionality of the library is based around the idea of a datatype dict containing a set of keys where each is a numpy array (or other iterable) where there is a 'ts' key, containing a numpy array of np.float64 timestamps, and then each iterable key should have the same number of elements (in the zeroth dimension) as the ts field. Thus a set of timestamped 'events' or other data type is defined. Other keys may be included which either aren't iterables or don't have the same number of elements in the zeroth dimension. These are therefore not interpreted as contributing dimensions to the set of data points. Concretely the datatypes which have some kind of support are:

- dvs
- frame
- sample
- imu
- pose6q
- point3
- flow
- skinSamples
- skinEvents
- ear
- cam

Definitions of minimal and optional(*) fields follow.

- fieldName   dimensions  datatype(numpy array unless otherwise stated) notes

## dvs:

- ts  n np.float64
- x   n np.uint16
- y   n np.uint16 As the sensor outputs it; plot functions assume that y increases in downward direction, following https://arxiv.org/pdf/1610.08336.pdf
- pol n np.bool To the extent possible, True means increase in light, False means decrease.
- dimX* 1 int
- dimY* 1 int

## frame:

- ts    n np.float64
- frame n list (of np.array of 2 or 3 dimensions np.uint8)

## sample:

- ts     n np.float64
- sensor n np.uint8
- value  n np.int16

## imu:

- ts  n    np.float64
- acc  nx3 np.float64 accelerometer readings [x,y,z] in m/s
- angV nx3 np.float64 angV readings [yaw, pitch roll?] in rad/s
- mag  nx3 np.float64 magnetometer readings [x, y, z] in tesla
- temp n   np.float64

## point3:

- ts    n   np.float64
- point nx3 np.float64 row format is [x, y, z]


## pose6q (effectively extends point3):

- ts       n   np.float64
- point    nx3 np.float64 row format is [x, y, z]
- rotation nx4 np.float64 row format is [rw, rx, ry, rz] where r(wxyz) define a quaternion

Note: quaternion order follows the convention of e.g. blender (wxyz) but not e.g. ros. (xyzw)

## flow: (per-pixel flow events)

- ts  n np.float64
- x   n np.uint16
- y   n np.uint16  
- vx   n np.uint16
- vy   n np.uint16  

## skinEvents: (intended for iCub neuromorphic skin events; could be generalised)

- ts n np.float64
- taxel n np.unit16
- bodyPart n np.uint8
- pol n np.bool

## skinSamples: (intended for dense iCub skin samples; could be generalised)

- ts n np.float64
- pressure nxm np.float64; m is the number of taxels concurrently sampled. Note:
there exist in the wild examples where the pressure value is a raw 8-bit sample.

## ear: (intended for cochlea events from UDS / Gutierrez-Galan, could be generalised)

- ts n np.float64
- freq n np.uint8
- pol n np.bool
- (There follow a number of model-specific fields which contribute to the full address: xsoType, auditoryModel, itdNeuronIds)

## cam:

Following ros msg camera info, the fields this might contain include:

- height           1   int
- width            1   int
- distortion_model     string
- D                5   np.float64 distortion params
- K                3x3 np.float64 Intrinsic camera matrix
- R                3x4 np.float64 Rectification matrix
- P                4x4 np.float64 projection matrix
