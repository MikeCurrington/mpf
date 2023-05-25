"""Interface defines for CANPIN."""

class CanPinMessages:
    """CANPIN messages."""

    Ping = 0
    DeviceReady = 1
    WelcomeNewHost = 2
    AssignDeviceIndex = 3
    GetNodeCapabilities = 4

    SetBoardInputPins = 5
    SetBoardOutputPins = 6  
    SetBoardLedPins = 7
    StartGpio = 8             # Request to 'apply' the input/output/led pins and start processing.

    ChangedInputs = 10        # Sent to host by all other boards when inputs change
    SetRecycleTime = 11
    SetMaxPulseTime = 12
    SetOutput = 13
    ResetOutput = 14
    ConfigureHardwareRule = 15
