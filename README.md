
# Sweeping Demos

This repository holds various small demos that make a Boston Dynamics Spot robot sweep the ground in various ways.

## Demos
 - `./spot_sweep_touch.py` This demo has Spot sweep back and forth directly in front of it, move forward, sweep back and forth directly in front of it, etc.
 - `./spot_sweep_touch_in_bounds.py` This demo makes Spot look for two fiducials on the ground, assumes that they define a square of ground that must be swept, and makes Spot sweep the area in an S-shape while touching the ground the whole time.
 - `./spot_sweep_hover.py` This demo works much like `./spot_sweep_touch.py` but makes Spot's hand hover over the ground as it sweeps it.
 - `./spot_sweep_hover_in_bounds.py` This demo works much like `./spot_sweep_touch_in_bounds.py`, but makes Spot's hand hover over the ground rather than touch it as it sweeps it.


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

IMPORTANT: It is better to use the tablet to "hijack" Spot when it starts becoming dangerous than it is to cut its power. Hijacking Spot (forcefully taking control of it via the tablet) will both stop the demo and allow you to manually pilot the robot back to a more suitable location. Using the E-Stop should be reserved for immediate emergencies as it cuts the power to Spot and makes it violently fall to the ground.

At this point, running `python spot_sweep_touch.py` (or running any of the other demos in a similar manner) should prompt you to input your username and password for Spot. After inputting your username and password, the demo should run and make Spot do as described in the `Demos` section of this README.

## Dependencies

 - The Boston Dynamics Spot SDK
 - PIL
 - scipy
 - numpy

## Other Files
 - `./spot_api.py` The Spot SDK provides many methods to control Spot, but in a way that is often confusing and untyped. As such, this file consolidates much of the SDK's functionality into a higher-level API that is easier to use and understand.
 - `./spot_motion_capture_to_video.py` This script takes a pre-recorded video of a sweeping motion and colors in the area that Spot sweeps in it. The video must be in black and white and must contain IR motion capture tags that are significantly lighter than the rest of the scene.

 ## Example Video

![Video of Spot Sweeping the Ground](./assets/SpotSweepingExample.gif)




