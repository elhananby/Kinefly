# Kinefly

Kinefly extracts kinematics variables from the camera image of a tethered winged insect.

You may track the movement of each bodypart: head, abdomen, left wing, right wing. Several tracking techniques are available to choose from (area tracking, edge finding, tip tracking). An auxiliary elliptical region may also be monitored (intended to capture leg motion or wingbeat frequency).

The software runs on ROS, the Robot Operating System, and can use any camera or other source that provides a ROS image stream, e.g. `/camera/image_raw`.

## Table of Contents

- [Kinefly](#kinefly)
  - [Table of Contents](#table-of-contents)
  - [Docker Setup](#docker-setup)
    - [Prerequisites](#prerequisites)
    - [Building the Docker Image](#building-the-docker-image)
    - [Running Kinefly in Docker](#running-kinefly-in-docker)
  - [Classic Installation (NOT RECOMMENDED)](#classic-installation-not-recommended)
  - [Running](#running)
  - [Main Window](#main-window)
  - [Handle Positioning](#handle-positioning)
  - [Output Variables](#output-variables)
    - [Output Structure](#output-structure)
    - [Example Output](#example-output)
    - [Field Descriptions](#field-descriptions)
  - [Units](#units)
  - [Bodypart Trackers](#bodypart-trackers)
    - [Area Tracking](#area-tracking)
    - [Edge Tracking](#edge-tracking)
    - [Tip Tracking](#tip-tracking)
    - [Intensity Tracking and Wingbeat Frequency](#intensity-tracking-and-wingbeat-frequency)
  - [Frame Rate](#frame-rate)
  - [White On Black](#white-on-black)
  - [Background Subtraction](#background-subtraction)
  - [Parameters](#parameters)
    - [General Parameters](#general-parameters)
    - [Head/Abdomen/Left/Right Parameters](#headabdomenleftright-parameters)
    - [Area Tracking Parameters](#area-tracking-parameters)
    - [Edge \& Tip Tracking Parameters](#edge--tip-tracking-parameters)
    - [Aux Parameters](#aux-parameters)
  - [Voltage Output](#voltage-output)
    - [PhidgetsAnalog Parameters](#phidgetsanalog-parameters)
    - [Voltage Calculation](#voltage-calculation)
    - [Example](#example)
  - [Record Video](#record-video)
  - [Retrack Prerecorded Video](#retrack-prerecorded-video)
  - [LED Panels Control](#led-panels-control)
  - [ROS Commands](#ros-commands)
  - [Plot Tracker](#plot-tracker)

## Docker Setup

For those who prefer to use Docker, we provide a containerized version of Kinefly. This can help avoid compatibility issues and simplify the setup process.

### Prerequisites

- Docker installed on your system
- X11 server running on your host machine (for GUI applications)

### Building the Docker Image

1. Navigate to the Kinefly repository:
   ```bash
   cd path/to/Kinefly
   ```

2. Build the Docker image:
   ```bash
   docker build -t kinefly:16.04 -f docker/Dockerfile .
   ```

### Running Kinefly in Docker

1. Allow the Docker container to access your X server:
   ```bash
   xhost +local:docker
   ```

2. Run the Kinefly container:
   ```bash
   docker run -it \
     --privileged \
     --net=host \
     -e DISPLAY=$DISPLAY \
     -e RIG=yourrigname \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -v /dev/bus/usb:/dev/bus/usb \
     -v /path/to/your/config:/root/catkin/src/Kinefly/launch/yourrigname \
     kinefly:16.04
   ```

   Replace `yourrigname` with your actual rig name and `/path/to/your/config` with the path to your rig-specific configuration folder.

3. When you're done, revoke the X server access:
   ```bash
   xhost -local:docker
   ```

This Docker setup allows you to run Kinefly in a contained environment while still accessing your system's USB devices and display. It's particularly useful for ensuring consistent behavior across different systems.

## Classic Installation (NOT RECOMMENDED)
You need:
- ROS
- A camera to supply a ROS image stream
- [Kinefly](https://github.com/ssafarik/Kinefly)
- [ledpanels](https://github.com/ssafarik/ledpanels)
- [phidgets](https://github.com/ssafarik/phidgets)

The Kinefly code is independent of the specific hardware and settings that you're using. This is accomplished by using parameters that are specific to your hardware (i.e. RIG). The parameters are set in a subdirectory of the Kinefly/launch directory, and each machine sets an environment variable 'RIG' that tells Kinefly which set of parameters to use.

If you want to output voltages (e.g. voltages to indicate wing angles, etc), then you'll need the PhidgetsAnalog hardware w/ Phidgets API. If you want to have the Kinefly measurements control LED panels, then you'll need panels hardware.

Refer to the file 'install.md' for detailed instructions.

## Running

To run Kinefly, you first need to ensure that roscore is running. In a terminal window:

```bash
roscore
```

Next, depending on if you have an ethernet or firewire or USB camera, edit the `Kinefly/launch/RIG/source_camera.launch` file as appropriate (where 'RIG' is the name of your rig). You may also need to edit the 'params_camera.launch' or 'params_kinefly.launch' files to set exposure, framerate, tracking params, etc.

Then in another terminal window use the roslaunch command to launch Kinefly:

```bash
roslaunch Kinefly main.launch
```

## Main Window

The main window provides an interface to most, but not all, of the functionality. The buttons and checkboxes on the toolbar, and the handles on the image, function as follows:

- `<Exit>`: Terminate the program. Use this instead of the titlebar's [x].
- `<SaveBG>`: Save the instant camera image to disk as ~/kinefly.png
- Track:
  - `[]H`: Turn on/off head tracking.
  - `[]A`: Turn on/off abdomen tracking.
  - `[]L`: Turn on/off left wing tracking.
  - `[]R`: Turn on/off right wing tracking.
  - `[]X`: Turn on/off the 'aux' area intensity & wingbeat frequency calculations.
- Subtract:
  - `[]H`: Turn on/off background subtraction on head
  - `[]A`: Turn on/off background subtraction on abdomen
  - `[]LR`: Turn on/off background subtraction on left & right wings
  - `[]X`: Turn on/off background subtraction on aux
- `[]Stabilize`: Turn on/off image stabilization for head & abdomen. The quality of stabilization is very dependent upon the region selected with the handles. (default: false)
- `[]Symmetric`: When moving handles, keep them symmetric about the body axis.
- `[]Windows`: Turn on/off helpful windows showing various internal images.

## Handle Positioning

Each of the head/abdomen/left/right body parts has a set of control points that determine what part of the image is tracked.  The handle positions determine the hinges, angles, and radii.

First, for best tracking, each of the head/abdomen/left/right body parts should have the 'hinge' handle placed at the center of rotation, which for example in the case of the abdomen is not the center of the abdomen, but rather is the point where the abdomen connects to the rest of the body, and rotates about.

For the head and abdomen, set the inner and outer radii to cover just a narrow strip that encloses unique features of the bodypart as the bodypart rotates, and set the angles to cover all of the bodypart motion.  Reliable head rotation seems to be the most dependent on proper handle positions.  

For the wings, the angles and radii should be positioned to indicate a region of interest such that legs and other non-wing moving things do not encroach into the region of interest.

## Output Variables

The output is published on the ROS topic `/kinefly/flystate`. You can retrieve this data in two ways:

1. Running the command:
   ```bash
   rostopic echo kinefly/flystate
   ```
2. Writing your own ROS node and subscribing to that topic.

### Output Structure

The fields shown in brackets `[]` are lists of values. There may be zero, one, or more than one entry (e.g., when finding multiple wing edges). Which fields contain valid values depends on which bodypart tracker has been set and its settings.

### Example Output

Here's an example of the `flystate` output:

```yaml
header: 
  seq: 1033
  stamp: 
    secs: 1393885525
    nsecs: 419325113
  frame_id: Fly
head: 
  angles: [-0.05834060178390743]
  gradients: []
  radii: [54.635968488535795]
  freq: 0.0
  intensity: 0.45413453997
abdomen: 
  angles: [-0.37292074013475407]
  gradients: []
  radii: [116.38383782940254]
  freq: 0.0
  intensity: 0.591699666712
left: 
  angles: [0.8233633148061248]
  gradients: [-0.00784313678741455]
  radii: []
  freq: 0.0
  intensity: 0.802197675672
right: 
  angles: [0.6048098145857046]
  gradients: [0.011551618576049805]
  radii: []
  freq: 0.0
  intensity: 0.593512067496
aux: 
  angles: []
  gradients: []
  radii: []
  freq: 0.0
  intensity: 0.673986792564
```

### Field Descriptions

- `header`: Standard ROS message header
- `head`, `abdomen`, `left`, `right`, `aux`: Body parts being tracked
  - `angles`: Rotation angles in radians
  - `gradients`: Intensity gradients
  - `radii`: Radial distances in pixels
  - `freq`: Frequency (e.g., wingbeat frequency)
  - `intensity`: Pixel intensity (range: 0.0 to 1.0)

Note: Empty lists `[]` indicate that no data was captured for that particular measurement.

## Units

Angle units are radians.  
Radius units are pixels.  
Intensity is on the range [0,1], where 0.0 equals all pixels black, and 1.0 equals all pixels white.  The main window shows intensity units in parentheses, ().
Gradient is the intensity change per unit, and is used for edge and tip finding.  Edge-tracking gradient is change per angle, and tip-tracking gradient is change per radial distance.

Each bodypart (head, abdomen, and wings) has its own angle origin, which in general terms is outward from the body center.  The zero angle for the head is directly opposite from the abdomen, and the zero of the abdomen angle is directly opposite from the head.  The zero for each wing is directly opposite from the other wing hinge.  In other words, imagine a line connecting the head and abdomen hinges, and a line connecting each wing hinge.  The zero angles are outward along those lines, and the range of each angle is on the range [-pi,+pi].

## Bodypart Trackers

The trackers available are as follows, and are specified via the 'tracker' parameter in the params_kinefly.launch file.  Any bodypart (except aux) can use any tracker.
'area'      Uses image registration techniques, and reports the rotation and
            radial movement of the area as a whole.
'edge'      Finds radial edges, ordered by decreasing edge strength.
'tip'       Finds the point where the bodypart is farthest from
            the rotation center.  
'intensity' Monitor the pixel intensity, and the frequency of periodic 
            intensity, of an elliptical area

There is an auxillary Plot Tracker tool that may be helpful for setting wing edge detection thresholds, etc.  See the Plot Tracker section of this document for details.

### Area Tracking

This is the default for the head & abdomen.  We measure rotation and radial expansion of the bodypart via an image registration technique in polar coordinates on the region of interest.  This state measurement is autozero'ed such that the origin tends to be near the midpoint of the range of motion.  Autoranging will therefore cause occasional changes of the angle origin, making it seem as though there were a step change in the angle when there wasn't.  Autoranging can be turned off by changing the parameter setting.  

Area tracking is also affected by a "feathering" (i.e. a window function) parameter that smooths the edge pixels of the bodypart.  Setting feathering=0 gives a rectangular window (i.e. a hard edge), and feathering=1 gives a Hanning window (i.e. a very smooth edge).  Tracking of small body parts, such as a fly head, may benefit from setting feathering=0.  The default is 0.25.

### Edge Tracking

This is the default for left & right wings.  We find multiple edge angles per wing (you can use this feature to track legs etc too, if desired), the number of edges being specified via parameter 'n_edges_max'.  The edge angles are in one list, and their corresponding intensity gradients are in another.  The edges are found by looking at the intensity gradient across the wingstroke, and each list is sorted in descending order of absolute gradient.

The parameter 'threshold' can be used to reduce false detections when no edge is present, and will typically be a low value such as 0.01.  It is set independently for each wing.  

### Tip Tracking

When the 'tip' tracker is set for a bodypart, We locate the bodypart point farthest from the point of rotation.  The parameter 'threshold' is compared to the radial gradient to find the bodypart, and should be set to approx 0.01. 

### Intensity Tracking and Wingbeat Frequency

We make an attempt to measure the wingbeat frequency using the frequency spectrum of the aux intensity.  For this to work, several criteria must be met.  First, you must set the valid range of wingbeat frequencies in the params_kinefly.launch file.  Upon the next launch, Kinefly will print out the compatible camera framerate needed (the lowest typically being slightly higher than twice the wingbeat passband width).  Next, set the camera to that framerate.  If the camera framerate can be maintained within the valid range, then Kinefly will compute the wingbeat frequency.  Since the framerate is typically much less than the Nyquist frequency needed to cover the highest wingbeat frequency, we use an undersampling technique to sample just the passband, and look at a baseband alias of the passband frequencies.  Proper camera framerate is critical to the functioning of this feature, and the computer must be fast enough to process the frames in the allowed time, with no camera framerate variation outside of the allowed range.  If you have trouble maintaining the framerate, consider using the parameter 'scale_image'.

## Frame Rate

The frame rate is dependent upon virtually every setting.  Turn off tracking of unneeded bodyparts, use the smallest usable regions of interest (i.e. handle positions), and turn off unneeded windows to speed the frame rate.  The largest improvement is likely to come from lowering the value of 'scale_image' in the params_kinefly.launch file.

## White On Black

Kinefly works best internally if the image is white-on-black.  The camera image may be either black-on-white or white-on-black, and whether to invert the color or not is decided automatically based on the pixel intensity of a region centered on the intersection of the lines head/abdomen hinges and left/right hinges.  If the mean pixel value of this region is darker than the mean pixel value of the overall image, then we invert the colors.

The user can override the autodetect and set the behavior manually via the checkbox on the toolbar.  When Kinefly launches it starts in autodetect mode, but once the checkbox is clicked, then the manual setting is used until the next launch of Kinefly.  The setting is not saved across launches.

## Background Subtraction

Background subtraction can be turned on/off for each bodypart individually.  The background is initialized with the kinefly.png image on disk, and updates as a moving average with the time constant (in seconds) parameter rc_background. 

Saturation of the background image can cause problems with tracking, due to limited foreground headroom.  Kinefly contains a "saturation correction" algorithm to reduce or eliminate this effect, however the feature is turned off by default due to the computational cost and reduced framerate.  You may turn it on/off for each bodypart separately via the parameter "saturation_correction".  The feature works by reducing the importance of pixels the closer they are to saturated.      

## Parameters

Parameters are stored in the file `Kinefly/launch/params_kinefly.launch`.

Anything set via the GUI (e.g., handle positions, checkbox states, etc.) is automatically saved to the `kinefly.yaml` file.

The non-GUI parameters are listed below:

### General Parameters

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| `filenameBackground` | Location to store the background image | `~/kinefly.png` |
| `image_topic` | ROS topic for the image stream | `/camera/image_raw` |
| `n_queue_images` | Length of the circular buffer for incoming images | 2 |
| `n_edges_max` | Max number of wing edges to find | 1 |
| `rc_background` | Time constant of the moving average background image | 1000.0 |
| `scale_image` | Scale all images by this amount. Smaller values give better framerate | 1.0 |
| `use_gui` | Turn on/off the graphical user interface to speed the framerate | `true` |
| `parameterfile` | Location of the .yaml file for keeping track of GUI handle positions & checkboxes | `~/kinefly.yaml` |

### Head/Abdomen/Left/Right Parameters

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| `tracker` | Which feature to track. Options are 'area', 'edge', and 'tip' | area/area/edge/edge |
| `saturation_correction` | Correct for bright pixels in the background image to improve tracking | `false` |

### Area Tracking Parameters

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| `autozero` | Automatically determine the zero angle based on center of motion (`true`), or use the first valid frame as the zero angle (`false`) | `true` |
| `feathering` | Set the amount of feathering for the edge pixels of the bodypart image. 0.0 gives a hard edge, 1.0 gives a very smooth edge | 0.25 |

### Edge & Tip Tracking Parameters

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| `threshold` | Threshold for the intensity gradient to detect a bodypart edge or tip | 0.0 |

### Aux Parameters

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| `wingbeat_min` | The lower wingbeat frequency (Hz) of your insect | 180.0 |
| `wingbeat_max` | The upper wingbeat frequency (Hz) of your insect | 220.0 |

**Note:** For the `image_topic` parameter, running the ROS node `image_proc` in addition to Kinefly and setting this to `image_mono` may help with some cameras that provide Bayer encoded images.

**Tip:** You may run the program `plot_tracker.py` for help setting the `threshold` parameter. See the [Plot Tracker](#plot-tracker) section of this document for details.


## Voltage Output

Kinefly can output voltages using a PhidgetsAnalog device via the ROS node `flystate2phidgetsanalog`. This node should be launched under the ROS namespace of kinefly:

```xml
<node name="flystate2phidgetsanalog" ... ns="kinefly" />
```

By default, the four voltage channels are set to (L, R, L-R, L+R), with the voltage value equal to the angle in radians.

### PhidgetsAnalog Parameters

PhidgetsAnalog parameters are specified in the `params_kinefly.launch` file. These parameters determine how the four voltage output channels set their voltages.

| Parameter | Description |
|-----------|-------------|
| `v#enable` | Enable/disable each channel separately (true/false) |
| `autorange` | Automatically set the four channels to L, R, L-R, L+R such that the voltages cover the range [-10,+10] based on the min/max witnessed angles |

Where `#` is one of 0, 1, 2, or 3, corresponding to each voltage channel.

### Voltage Calculation

Each voltage channel calculates its output using a linear combination of terms. The general formula is:

$$
v_\text{#out} = v_\text{#l1} \cdot \text{left.angles[0]} + v_\text{#l2} \cdot \text{left.angles[1]} + \\
v_\text{#r1} \cdot \text{right.angles[0]} + v_\text{#r2} \cdot \text{right.angles[1]} + \\
v_\text{#ha} \cdot \text{head.angles[0]} + v_\text{#hr} \cdot \text{head.radii[0]} + \\
v_\text{#aa} \cdot \text{abdomen.angles[0]} + v_\text{#ar} \cdot \text{abdomen.radii[0]} + \\
v_\text{#xi} \cdot \text{aux.intensity}
$$

Where `#` represents the channel number (0, 1, 2, or 3).

### Example

To generate a voltage output such as L-R on channel 2:

- Set `v2l1 = +1`
- Set `v2r1 = -1`

This configuration will output the difference between the left and right wing angles on voltage channel 2.
## Record Video

During a Kinefly session, you may record a video to a .bag file, for later analysis, with a timestamped name such as `~/bagfiles/_2014-06-16-14-53-28.bag`:
```bash
roslaunch Kinefly record.launch
```
You may also attach a prefix to the filename, such as '~/bagfiles/flynumberone_2014-06-16-14-53-28.bag':
```bash
roslaunch Kinefly record.launch prefix:=flynumberone
```
## Retrack Prerecorded Video

If environment var RETRACK is set to 1, and BAGFILE is set to a file (i.e. `export RETRACK=1` and `export BAGFILE=/path/to/file.bag`), then it uses the given .bag file instead of the camera(s).

An example, to retrack a single .bag file:       
```bash                                    
export RETRACK=1                                      
export BAGFILE=/home/rancher/bagfiles/12345.bag       
roslaunch Kinefly main.launch                         
```

To retrack a bunch of bag files, use the Kinefly/retracker program.  An example, to retrack multiple .bag files:
```bash
export BAGFILESPEC=/home/rancher/bagfiles/*.bag       
rosrun Kinefly retracker
```

## LED Panels Control

Kinefly can control the MReiser LED Panels (see bitbucket.org/mreiser/panels, and github.com/ssafarik/ledpanels) either directly over USB, or indirectly via the voltage signal generated by the voltage output above.

In either case the ROS node from github.com/ssafarik/ledpanels must be running in order to send serial commands to the panels.  If you want to control the panels via USB, then you'll also need to run the ROS node flystate2ledpanels (which should be launched under the ROS namespace of kinefly, i.e. `<node name="flystate2ledpanels ... ns="kinefly" />)`.
 
LEDPanels parameters are specified in the 'params_kinefly.launch'.  The parameters determine how the four voltage ADC input channels interpret the voltages into position and velocity commands. xpos, xvel, ypos, and yvel are determined by a set of adc# coefficients, where # is one of 0|1|2|3.

Each x or y channel calculates a position or velocity by a linear combination of terms of the form:

$Q = \text{adc0} \cdot \text{bnc0} + \text{adc1} \cdot \text{bnc1} + \text{adc2} \cdot \text{bnc2} + \text{adc3} \cdot \text{bnc3} + \text{funcx} \cdot f_x(t) + \text{funcy} \cdot f_y(t)$

where $fx(t)$ and $fy(t)$ are the internal panel controller functions that default to constant values of 10, unless otherwise changed.

The value Q then applies to either x or y as specified by the 'axis' parameter, and is interpreted as a position or velocity as specified by the 'mode' parameter.

## ROS Commands

The kinefly/command ROS topic accepts the following string commands:

- `help`: Shows this message.
- `gui_on`: Turn on the graphical user interface.
- `gui_off`: Turn off the graphical user interface.
- `exit`: Exit the program.

You can send the above commands at the shell prompt, for example:

```bash
rostopic pub -1 kinefly/command Kinefly/MsgCommand commandtext help
rostopic pub -1 kinefly/command Kinefly/MsgCommand commandtext gui_on
rostopic pub -1 kinefly/command Kinefly/MsgCommand commandtext gui_off
rostopic pub -1 kinefly/command Kinefly/MsgCommand commandtext exit
```

## Plot Tracker

The program plot_tracker.py will open a plot window that may help in analysing the performance of the kinematics tracking.  The plot shows a live view of internal tracker data, such as intensity profiles and thresholds, and this information may help you in setting various tracker parameters, such as threshold.

Each Kinefly tracker (edge, tip, etc) makes its data available via a service name such as 'trackerdata_head', 'trackerdata_left', 'trackerdata_right', or 'trackerdata_abdomen'.  

Prior to running plot_tracker, set the parameter 'trackername' to the signal of interest.
For example:
```bash
rosparam set trackername trackerdata_left
rosrun Kinefly plot_tracker.py
```