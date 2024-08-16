"""
Small Demo for Spot.
"""
import asyncio
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from spot_api import Spot

async def main():
    """
    Identifies a square area on the ground designated by two fiducials.
    """

    # create the Spot robot
    spot = Spot()

    # Ensure that Spot has an arm.
    assert spot.has_arm(), "Spot must have an arm for this demo."

    # Lease the robot
    with spot.leased(take=True) as spot:

        # Power the motors of the Spot on
        await spot.power_motors_on()

        # Make Spot stand up
        await spot.stand()

        # get the fiducials
        fiducials = await spot.fiducials_in_view()

        # if we have too many or too few we ask the user to add more or disambiguate before trying again
        if len(fiducials) != 2:
            print(f"Exactly 2 fiducials must be in view. Instead, {len(fiducials)} were in view.")
            return

        # get the fiducials
        f1 = fiducials[0]
        f2 = fiducials[1]

        # get the poses of the fiducials in the `ODOM` frame
        pose_f1 = f1.odom_t_fiducial()
        pose_f2 = f2.odom_t_fiducial()

        # check that the fiducials are on the same horizontal plane (within a given threshold)
        THRESH = 1
        if not (-THRESH <= (pose_f1.z - pose_f2.z) <= THRESH):
            print("Detected a large vertical difference between the two fiducials. They must be on the floor.")
            return

        # Generate points
        # The points are generated under the assumption that the scan area was
        # right in front of spot when it booted up and therefore positive y is
        # forward (relative to the bot) and positive x is to the right (relative
        # to the bot).

        scan_height = 0.2 # how many meters tall the scan line covers
        #scan_width  = 0.5 # how many meters wide the scan line covers

        min_x = min(pose_f1.x, pose_f2.x)
        min_y = min(pose_f1.y, pose_f2.y)
        max_x = max(pose_f1.x, pose_f2.x)
        max_y = max(pose_f1.y, pose_f2.y)

        i = 0
        points = []
        while (y := (min_y + (i*(scan_height/2)))) <= max_y:
            if i % 2 == 0:
                points.append((min_x, y))
                points.append((max_x, y))
            else:
                points.append((max_x, y))
                points.append((min_x, y))
            i += 1

        # Make Spot sweep the area
        await spot.arm_floor_contact(
            points=points,
            frame_name=ODOM_FRAME_NAME,
            press=0.01,
            body_can_move=True
        )

        # stow the arm
        await spot.stow_arm(extra_safe=True)

if __name__ == "__main__":
    asyncio.run(main())
