import asyncio
from itertools import chain
import time
from typing import Generator, Literal
from spot_api import Spot
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME, GROUND_PLANE_FRAME_NAME, get_a_tform_b
from bosdyn.client.math_helpers import Quat, SE3Pose, Vec3, math

Num = float | int

HEIGHT: Num = 0.2 # how far off the ground to keep the GRIPPER
SCANNER_LEN: Num = 0.2 # the scanner's length (i.e. this is the vertical width of the scan area)
F1: int =  10 # the number of the top   -left  fiducial
F2: int =  9  # the number of the bottom-right fiducial
VELOCITY: Num  = 0.1 # the velocity of the scan
# How to align the Scan Box
#   The two fiducials define the top-left and bottom-right corner of the are we
#   are scanning, but there are multiple squares that go through them. As such,
#   we have a decision as to what angle the scan box should be placed at.
#   If it is "f1", then the box will be aligned with the rotation of the `F1` fiducial.
#   If it is "f2", then the box will be aligned with the rotation of the `F2` fiducial.
#   If it is "body", then the box will be aligned with the rotation of the body when the program is begun.
ALIGN: Literal["f1", "f2", "body"] = "f1"


async def main():
    """
    Identifies a square area on the ground designated by two fiducials.
    """

    # create the Spot robot
    spot = Spot()

    # Ensure that Spot has an arm.
    assert spot.has_arm(), "Spot must have an arm for this demo."

    #print(await spot.robot_state())
    #print(await spot.robot_hardware())
    #print(await spot.robot_metrics())

    # Lease the robot
    with spot.leased(take=True) as spot:

        # Power the motors of the Spot on
        await spot.power_motors_on()

        # Make Spot stand up
        await spot.stand()

        time.sleep(1)

        # get the fiducials
        fiducials = await spot.fiducials_in_view()

        # if we have too many or too few we ask the user to add more or disambiguate before trying again
        if len(fiducials) != 2:
            print(f"Exactly 2 fiducials must be in view. Instead, {len(fiducials)} were in view.")
            return

        # get the fiducials
        f1 = fiducials[0]
        f2 = fiducials[1]

        if (f2.id == F1) or (f1.id == F2):
            (f1, f2) = (f2, f1)
        if f1.id != F1:
            print(f"fiducial {F1} was not seen, fiducial {f1.id} seen instead")
        if f2.id != F2:
            print(f"fiducial {F2} was not seen, fiducial {f2.id} seen instead")

        print(f"TL: {f1.id}, BR: {f2.id}")

        ## get the fiducial in the odom frame
        odom_t_f1 = f1.odom_t_fiducial()
        odom_t_f2 = f2.odom_t_fiducial()

        # check that the fiducials are on the same horizontal plane (within a given threshold)
        THRESH = 1
        if not (-THRESH <= (odom_t_f1.z - odom_t_f2.z) <= THRESH):
            print("Detected a large vertical difference between the two fiducials. They must be on the floor.")
            return

        # Get Points in Accordance to their rotation frame so that the box has the correct alignment

        body_t_f1 = f1.body_t_fiducial()
        body_t_f2 = f2.body_t_fiducial()

        if   ALIGN == "f1":
            # Put everything in `F1` frame
            f1_t_body = body_t_f1.inverse()
            f1_pose   = f1_t_body * body_t_f1
            f2_pose   = f1_t_body * body_t_f2
        elif ALIGN == "f2":
            # Put everything in `F2` frame
            f2_t_body = body_t_f2.inverse()
            f1_pose   = f2_t_body * body_t_f1
            f2_pose   = f2_t_body * body_t_f2
        else: # ALIGN == "body"
            # Put everything in "body" frame
            f1_pose = body_t_f1
            f2_pose = body_t_f2

        # Generate points
        # The points are generated under the assumption that the scan area was
        # right in front of spot when it booted up and therefore positive y is
        # forward (relative to the bot) and positive x is to the right (relative
        # to the bot).

        # get the "box" bounds for the rectangle we want to scan within
        sort_args = [f1_pose, f2_pose]
        min_x = min(*sort_args, key=lambda v: v.x).x # bottom
        min_y = min(*sort_args, key=lambda v: v.y).y # right
        max_x = max(*sort_args, key=lambda v: v.x).x # top
        max_y = max(*sort_args, key=lambda v: v.y).y # left

        def frange(start: float, end: float, step: float) -> Generator[float, None, None]:
            """Float version of `range(..)`."""
            assert step != 0, "step must not be 0"
            if end < start:
                step = -abs(step)
                start += step
                while end < start:
                    yield start
                    start += step
            else:
                step =  abs(step)
                while start < end:
                    yield start
                    start += step

        # generate the points to scan
        points = []
        for (i, x) in enumerate(frange(min_x - SCANNER_LEN/2, max_x - SCANNER_LEN/2, SCANNER_LEN/2)):
            if i % 2 == 0:
                points.append((x, min_y))
                points.append((x, max_y))
            else:
                points.append((x, max_y))
                points.append((x, min_y))

#        print((max_x, max_y), (min_x, min_y))
#        import pprint
#        pprint.pprint(points)
#        return

        # translate points to odom frame
        if   ALIGN == "f1":
            odom_t_align = odom_t_f1
        elif ALIGN == "f2":
            odom_t_align = odom_t_f2
        else: # ALIGN == "body"
            f1_t_body    = body_t_f1.inverse()
            odom_t_align = odom_t_f1 * f1_t_body

        npoints: list[tuple[float, float, float]] = []
        for (x, y) in points:
            align_t_point = SE3Pose(x, y, f1_pose.z, f1_pose.rot)
            odom_t_point  = odom_t_align * align_t_point
            if len(npoints) != 0:
                move_time = math.sqrt(
                    (npoints[-1][0] - odom_t_point.x)**2 +
                    (npoints[-1][1] - odom_t_point.y)**2
                )
            else:
                move_time = 1 # number of seconds to move to first position
            npoints.append((odom_t_point.x, odom_t_point.y, move_time / VELOCITY))
        points = npoints

        # get the rotation so that it is aligned with the scan area (which is aligned with the starting body rotation)
        align_t_rot = Quat.from_pitch(math.pi / 12)
        odom_t_rot  = odom_t_align.rot * align_t_rot

        # send the points
        for (x, y, move_time) in points:
            await spot.set_ee_pose(
                x, y, odom_t_f1.z + HEIGHT, # assume that the z is the z of the ground (because the fiducial should be on the ground)
                rot=odom_t_rot,
                frame_name=ODOM_FRAME_NAME,
                move_time=move_time,
                body_may_move=True,
            )

        # stow the arm
        await spot.stow_arm(extra_safe=True)

if __name__ == "__main__":
    asyncio.run(main())
