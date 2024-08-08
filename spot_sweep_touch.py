import asyncio
from spot_api import Spot


async def main():
    """
    Makes Spot sweep an infinitely-long corridor in front of him using a
    serpentine scan pattern. The number of times that Spot scans a line
    in front of him and then moves forward is currently limited, but could
    easily be made into an infinite loop.

    The gripper makes contact with the ground for the duration of the scan.
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

        F = 0.65
        h_len = 0.5

        # generate the points
        N = 100
        C = 0 # center point
        hline =      [(F, C + ((h_len/N) * i)) for i in reversed(range(1, int(N/2)))]
        hline.append( (F, C                  )                                       )
        hline.extend([(F, C - ((h_len/N) * i)) for i in          range(1, int(N/2)) ])

        points = []
        flip = False
        for i in range(3):
            points.extend([(x + (i * 0.2), -y if flip else y) for (x, y) in hline])
            flip = not flip

        # Make Spot's arm scan the floor in front of it.
        await spot.arm_floor_contact(
            points=points,
            press=0.01,
            velocity=0.2,
            await_floor_touch=True,
            body_can_move=True,
        )

        # reset the arm position or else it slams when it stows the arm
        await spot.set_ee_pose(x=0.8, z=0.5, rot="forward", move_time=2)

        # stow the arm
        await spot.stow_arm()


if __name__ == "__main__":
    asyncio.run(main())




