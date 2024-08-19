
# Sweeping Demos

This repository holds various small demos that make a Boston Dynamics Spot robot sweep the ground in various ways.

## Quick Start

In order to use this repository, you will first need to clone (via `git clone https://github.com/boston-dynamics/spot-sdk.git` or a similar command) the Spot SDK from boston dynamics and put it at the top level of this repo.

The repo will then look something like this:
```text
./spot-sdk/** # The downloaded Spot SDK
./spot_api.py # This repo's layer over the SDK
./spot_sweep_hover.py # Demo
... # The rest of the demos and files
```

Pip install all of the other dependencies found in the "Dependencies" section of this README.

Connect to Spot's wifi.

Make sure that Spot can power up its motors i.e. that the hardware button on Spot is not pressed to prevent Spot from powering up.

Pick up the touchpad provided with Spot and use it to drive Spot to a suitable location for the demo. For demos that require fiducials, it is best to have Spot standing and to verify via the tablet that it can see as many fiducials as are required for the demo (often 2 fiducials are required). Otherwise, it is not important whether Spot is stand or sitting at the start of the demo.

IMPORTANT: Spot's tablet functions as it's estop so be prepared to use the tablet to cut Spot's power if it is about to do something dangerous such as run into a person or wall. You can alternatively use the tablet to hijack (take back control of) the robot which will both stop the demo and allow you to manually pilot the robot back to a more suitable location. It is recommended to hijack the robot whenever possible as hijacking does not cut off Spot's power and therefore does not make Spot violently fall to the ground as cutting off its power does.

At this point, running `python spot_sweep_touch.py` (or running any of the other demos in a similar manner) should prompt you to input your username and password for Spot. After inputting your username and password, the demo should run and make Spot do as described in the `Demos` section of this README.

## Dependencies

 - The Boston Dynamics Spot SDK
 - PIL
 - scipy
 - numpy

## Demos
 - `./spot_sweep_touch.py` This demo has Spot sweep back and forth in front of it and then move forward a few times.
 - `./spot_sweep_touch_in_bounds.py` This demo makes Spot look for two fiducials on the ground, assumes that they define a square of ground that must be swept, and makes Spot scan the area in an S-shape.
 - `./spot_sweep_hover.py` This demo works much like `./spot_sweep_touch.py` but makes Spot's hand hover over the area as it scans it rather than put any pressure on the ground.
 - `./spot_sweep_hover_in_bounds.py` This demo works much like `./spot_sweep_touch_in_bounds.py`, but makes Spot's hand hover over the ground it is scanning rather than put constant pressure on it.
 - `./spot_motion_capture_to_video.py` This demo takes a pre-recorded video of a sweeping motion and colors in the area that Spot sweeps as it sweeps it.

## Other Files
 - `./spot_api.py` The Spot SDK provides many methods to control Spot, but in a way that is often confusing and untyped. As such, this file consolidates much of the SDK's functionality into a higher-level API that is easier to use and understand.

 ## Example Video

 [Video of Spot Sweeping the Ground](./motion_capture/assets/SpotSweepingExample.gif)




