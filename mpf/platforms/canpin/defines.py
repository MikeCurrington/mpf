"""Interface defines for CANPIN."""

class CanPinMessages:
    """CANPIN messages."""

    Ping = 0
    DeviceReady = 1
    WelcomeNewHost = 2
    AssignDeviceIndex = 3
    GetNodeCapabilities = 4

