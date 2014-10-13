RoAH Devices
============

This repository contains a ROS package designed to control
devices of the RoCKIn Home Automation Network directly.
When using this package, commands will **NOT** go through
the RoCKIn@Home Referee, Scoring and Benchmarking Box.

Here are provided:

- The roan_devices node, that provides ROS services and
topics to control the devices;
- A dummy server so that the real devices are not needed
for testing.


## Dependencies

You need to use at least ROS Hydro.

This was tested with Ubuntu 12.04.5 LTS (Precise Pangolin) and
14.04.1 LTS (Trusty Tahr).


## Compiling

Compile as a normal ROS package in your Catkin workspace.


## Running

Run the ROS node roah_devices or use the launch file for a quick test:
```bash
roslaunch --screen roah_devices control_dummy.launch
```


## Using

Output devices can be controlled by the available services:
```
/devices/blinds/max
/devices/blinds/min
/devices/blinds/set
/devices/dimmer/max
/devices/dimmer/min
/devices/dimmer/set
/devices/switch_1/off
/devices/switch_1/on
/devices/switch_1/set
/devices/switch_2/off
/devices/switch_2/on
/devices/switch_2/set
/devices/switch_3/off
/devices/switch_3/on
/devices/switch_3/set
```

The `set` services take an integer parameter: 0 or 1 for the switches
and a percentage from 0 to 100 for the dimmer and blinds. All other
services take no parameters and act simply as shortcuts to the
respective `set` service.

Services can easily be called from the command line:
```bash
rosservice call /devices/switch_2/on
rosservice call /devices/dimmer/set 50
```

Or from C++ code:
```c++
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "roah_devices/Bool.h"
#include "roah_devices/Percentage.h"



int main (int argc, char** argv)
{
  ros::init (argc, argv, "try_service_call");
  ros::NodeHandle nh;

  if (ros::service::waitForService ("/devices/switch_2/on", 100)) {
    std_srvs::Empty s;
    if (! ros::service::call ("/devices/switch_2/on", s)) {
      ROS_ERROR ("Error calling service");
    }
  }
  else {
    ROS_ERROR ("Could not find service");
  }

  if (ros::service::waitForService ("/devices/dimmer/set", 100)) {
    roah_devices::Percentage p;
    p.request.data = 75;
    if (! ros::service::call ("/devices/dimmer/set", p)) {
      ROS_ERROR ("Error calling service");
    }
  }
  else {
    ROS_ERROR ("Could not find service");
  }

  ros::spinOnce();
  return 0;
}
```
Don't forget that your ROS package must depend on std_srvs and roah_devices.

(Python should also be easy, contribution of an example is welcome.)
