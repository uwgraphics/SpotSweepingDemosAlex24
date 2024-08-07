import asyncio
from spot_api import Spot


async def main():
    """
    Makes Spot sweeps a horizontal area right in front of him and then takes a step to
    sweep the next area.
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

        p1 = (0.75, -0.5, -0.4)
        p2 = (0.75,  0.5, -0.4)

        await spot.set_ee_pose(
            x=p1[0],
            y=p1[1],
            z=p1[2],
            move_time=5
        )

        for _ in range(4):

            # scan the ground
            await spot.set_ee_pose(
                x=p2[0],
                y=p2[1],
                z=p2[2],
                move_time=2
            )

            # Move Spot forward
            await spot.move_forward(0.2, 1)

            # scan the ground
            await spot.set_ee_pose(
                x=p1[0],
                y=p1[1],
                z=p1[2],
                move_time=2
            )

            # Move Spot forward
            await spot.move_forward(0.2, 1)

            # Make Spot's arm scan the floor in front of it.
            #await spot.arm_floor_contact(
            #    [(0.75, 0.50), (0.75, 0.50)],
            #    -0.05, # do not touch the ground
            #)

        # Make spot sit.
        #await spot.sit()



if __name__ == "__main__":
    asyncio.run(main())




