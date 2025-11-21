# Client to send velocity commands to the Amiga robot.

class AmigaClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        # TODO: Initialize farm-ng / gRPC client here

    async def send_twist(self, linear_x: float, angular_z: float):
        """Send a twist command to the robot.

        Args:
            linear_x: Forward velocity in m/s.
            angular_z: Yaw rate in rad/s.
        """
        # TODO: Implement gRPC call to Amiga
        raise NotImplementedError("send_twist is not implemented yet")
