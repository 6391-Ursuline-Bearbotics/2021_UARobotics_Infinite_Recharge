{
    "controlType": "Talon",
    # Class names of motor controllers used.
    # Options:
    # 'WPI_TalonSRX'
    # 'WPI_TalonFX' (for Falcon 500 motors)
    # 'WPI_VictorSPX'
    # Note: The first motor on each side should always be a Talon SRX/FX, as the
    # VictorSPX does not support encoder connections
    "rightControllerTypes": ["WPI_TalonSRX", "WPI_TalonSRX"],
    "controllerTypes": ["WPI_TalonSRX", "WPI_VictorSPX"],
    # Note: The first id in the list of ports should be the one with an encoder
    # Ports for the left-side motors
    "motorPorts": [2, 1],
    # Ports for the right-side motors
    "rightMotorPorts": [4, 3],
    # Inversions for the left-side motors
    "motorsInverted": [False, False],
    # Inversions for the right side motors
    "rightMotorsInverted": [False, False],
    # Wheel diameter (in units of your choice - will dictate units of analysis)
    # .5802 is calculated wheel circum in meters
    "wheelDiameter": 0.5802,
    # If your robot has only one encoder, set all right encoder fields to `None`
    # Encoder edges-per-revolution (*NOT* cycles per revolution!)
    # This value should be the edges per revolution *of the wheels*, and so
    # should take into account gearing between the encoder and the wheels
    "encoderEPR": 16384,
    # Whether the left encoder is inverted
    "encoderInverted": True,
    # Whether the right encoder is inverted:
    "rightEncoderInverted": False,
    # Your gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
    "gyroType": "Pigeon",
    # Whatever you put into the constructor of your gyro
    # Could be:
    # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
    # "SerialPort.Port.kMXP" (MXP Serial port for NavX),
    # "I2C.Port.kOnboard" (Onboard I2C port for NavX),
    # "0" (Pigeon CAN ID or AnalogGyro channel),
    # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
    # "leftSlave" (Pigeon on the left slave Talon SRX/FX),
    # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
    "gyroPort": "3",
    "gearing": 1,
    "useIntegrated": True,
}







