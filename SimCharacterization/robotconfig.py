{
    # Ports for motors
    # If doing drive test, treat this as the left side of the drivetrain
    "motorPorts": [2,1],
    # Only if you are doing drive (leave empty "[]" if not)
    "rightMotorPorts": [4,3],
    # Class names of motor controllers used.
    # 'WPI_TalonSRX'
    # 'WPI_VictorSPX'
    # 'WPI_TalonFX'
    # If doing drive test, treat this as the left side of the drivetrain
    "controllerTypes": [WPI_TalonSRX, WPI_VictorSPX],
    # Only if you are doing drive (leave empty "[]" if not)
    "rightControllerTypes": [WPI_TalonSRX, WPI_TalonSRX],
    # Set motors to inverted or not
    # If doing drive test, treat this as the left side of the drivetrain
    "motorsInverted": [False, False],
    # Only if you are doing drive (leave empty "[]" if not)
    "rightMotorsInverted": [False, False],
    # Encoder edges-per-revolution (*NOT* cycles per revolution!)
    # For the CTRE Mag Encoder, use 16384 (4 * 4096 = 16384)
    "encoderEPR": 16384,
    # Gearing accounts for the gearing between the encoder and the output shaft
    "gearing": 1,
    # Encoder ports (leave empty "[]" if not needed)
    # Specifying encoder ports indicates you want to use Rio-side encoders
    # If doing drive test, treat this as the left side of the drivetrain
    "encoderPorts": [],
    # Only if you are doing drive (leave empty "[]" if not)
    "rightEncoderPorts": [],
    # Set to True if encoders need to be inverted
    # If doing drive test, treat this as the left side of the drivetrain
    "encoderInverted": True,
    # Only if you are doing drive (set to False if not needed)
    "rightEncoderInverted": False,
    # ** The following is only if you are using a gyro for the DriveTrain test**
    # Gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
    "gyroType": "AnalogGyro",
    # Whatever you put into the constructor of your gyro
    # Could be:
    # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
    # "SerialPort.Port.kMXP" (MXP Serial port for NavX),
    # "I2C.Port.kOnboard" (Onboard I2C port for NavX),
    # "0" (Pigeon CAN ID or AnalogGyro channel),
    # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
    # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
    "gyroPort": "1",
}

