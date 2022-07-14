## Usage

This package uses a serial (or serial over USB) port connection to read and control SimpleBGC (AlexMos) gimbals. It reads and publishes gimbal IMU and
encoder angles, which it converts to quaternions and publishes as seperate PoseStamped messages. 

- IMU angles published to: gimbal\_imu\_angles
- Encoder angles published to: gimbal\_enc\_angles

It also allows gimbal position or speed control. The user can publish a PoseStamped message to gimbal\_target\_orientation for position control. Alternatively, they can publish a TwistStamped message to gimbal\_target\_speed for speed control. If both topics receive messages simultaneously, speed control takes precedence. If no message is received on the target speed topic for over 0.5s, the gimbal is set to stop moving.
Gimbal angle limits can also be optionally passed in, default values are listed below under Params. Note, setting these should prevent the gimbal from exceeeding mechanical limits, as long as the gimbal coordinate system is set to default SBGC configurations. It is recommended that users test each axis corresponds to expected motion by sending small orientation change or speed requests to the gimbal. For example, if "z" is expected to be yaw, the user can send a small angular speed command of 0.5deg/s about the z axis and confirm whether the correct gimbal motor moves.

## Installation

Clone repository to ROS workspace, build workspace using catkin build, source, and then use the ros_sbgc_driver.launch file to run the driver.

## Params

* Rate
  * Int, frequency (Hz) to run node at. May be limited by gimbal baud rate.
  * Default: 30
* Baud Rate
  * Int, baud rate
  * Default: 115200
* PrintAngles
  * Bool, to display IMU+ENC angles on terminal.
  * Default: False
* Device Path
  * String, path to device.
  * Default: "/dev/ttyUSB0".
* GimbalLimits (doubles)
  * maxRoll_degs (default: 0)
  * minRoll_degs (default: 0)
  * maxPitch_degs (default: 90)
  * minPitch_degs (default: -90)
  * maxYaw_degs (default: 90)
  * minYaw_degs (default: -90)

## Sample Commands

### Sample Speed Target command:

```
 rostopic pub -r 5 /gimbal_target_speed geometry_msgs/TwistStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0         
    y: 0.0            
    z: 9.0"
```

The command above publishes the TwistStamped message at 5Hz, and causes the connected gimbal to yaw at 9deg/s. When the above command is terminated, the driver will automatically stop the gimbal after 0.5s.

### Sample Angle Target command:

```
rostopic pub -r 2 /gimbal\_target\_orientation geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 1.103014e-07
    y: 0.0001917475655
    z: -0.000575242753
    w: 0.9999999" 
```

The command above publishes the PoseStamped message at 2Hz, and moves the gimbal to the orientation specified by the quaternion.
