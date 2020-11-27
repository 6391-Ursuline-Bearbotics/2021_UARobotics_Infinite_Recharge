{
    # Type of control:
    # 'SparkMax_Brushed' (for using CAN spark maxes, brushed)
    # 'SparkMax_Brushless' (for using CAN spark maxes,brushless)
    # **use CANSparkMax whenever using the SparkMax control**
    # 'Talon' (for using CAN based CTRE products)
    # 'Simple' (for using any other motor controllers)
    "controlType": "Simple",
    # Ports for motors
    # if doing drive test, treat this as the left of the drivetrain
    "motorPorts": [8,9],
    # only if you are doing drive (set to None if not)
    "rightMotorPorts": [],
    # Class names of motor controllers used.
    # Options:
    # 'Spark'
    # 'Victor'
    # 'VictorSP'
    # 'PWMTalonSRX'
    # 'PWMVictorSPX'
    # 'WPI_TalonSRX'
    # 'WPI_VictorSPX'
    # 'WPI_TalonFX'
    # 'CANSparkMax' (Use this when using a Spark Max control mode)
    "controllerTypes": ["WPI_VictorSPX", "WPI_VictorSPX"],
    # only if you are doing drive (set to None if not)
    "rightControllerTypes": [],
    # set motors to inverted or not
    "motorsInverted": [True, False],
    # only if you are doing drive (leave empty if not)
    "rightMotorsInverted": [],
    # If your robot has only one encoder, set all right encoder fields to `None`
    # Encoder edges-per-revolution (*NOT* cycles per revolution!)
    # This value should be the edges per revolution *of the wheels*, and so
    # should take into account gearing between the encoder and the wheels
    "encoderEPR": 8192,
    # encoder ports (if needed)
    "encoderPorts": [0, 1],
    # only if you are doing drive (set to None if not)
    "rightEncoderPorts": [],
    # Ports for encoders (if needed)
    "encoderInverted": True,
    # only if you are doing drive (set to None if not needed)
    "rightEncoderInverted": None,
    # Your gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
    "gyroType": None,
    # Whatever you put into the constructor of your gyro
    # Could be:
    # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
    # "SerialPort.Port.kMXP" (MXP Serial port for NavX),
    # "I2C.Port.kOnboard" (Onboard I2C port for NavX),
    # "0" (Pigeon CAN ID or AnalogGyro channel),
    # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
    # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
    "gyroPort": "",
    "gearing": 1,
    # (Only for neo's) Choose if you want to use the integrated NEO
    # encoder or if you want to use a quadrature encoder
    "useIntegrated": False,
}


