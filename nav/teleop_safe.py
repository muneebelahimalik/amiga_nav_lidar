# Main entrypoint: keyboard teleop + LiDAR-based safety + Amiga client.

import asyncio

from amiga.amiga_client import AmigaClient
from nav.safety_layer import apply_safety

SAFE_DISTANCE = 1.5  # meters


async def main():
    """Run safety-aware teleoperation loop.

    This is a skeleton; fill in LiDAR and keyboard parts.
    """
    client = AmigaClient(host="127.0.0.1", port=8002)

    while True:
        # TODO: Replace with real keyboard input
        v_des, w_des = 0.0, 0.0

        # TODO: Plug in LiDAR processing here
        d_min = None  # Placeholder: no LiDAR yet

        v_cmd, w_cmd = apply_safety(v_des, w_des, d_min, safe_distance=SAFE_DISTANCE)

        await client.send_twist(v_cmd, w_cmd)
        await asyncio.sleep(0.05)


if __name__ == "__main__":
    asyncio.run(main())
