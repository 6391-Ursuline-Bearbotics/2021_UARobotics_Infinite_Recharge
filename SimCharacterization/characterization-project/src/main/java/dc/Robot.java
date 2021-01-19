/**
* This is a very simple robot program that can be used to send telemetry to
* the data_logger script to characterize your drivetrain. If you wish to use
* your actual robot code, you only need to implement the simple logic in the
* autonomousPeriodic function and change the NetworkTables update rate
*/

package dc;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// WPI_Talon* imports are needed in case a user has a Pigeon on a Talon
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;
import com.revrobotics.AlternateEncoderType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;

import java.util.ArrayList; 

public class Robot extends TimedRobot {

  static private double ENCODER_EDGES_PER_REV = 16384 / 4.;
  static private int PIDIDX = 0;
  static private int ENCODER_EPR = 16384;
  static private double GEARING = 1;
  
  private double encoderConstant = (1 / GEARING) * (1 / ENCODER_EDGES_PER_REV);

  Joystick stick;
  DifferentialDrive drive;

  WPI_TalonSRX leftMotor = setupWPI_TalonSRX(2, Sides.LEFT, false);
  WPI_TalonSRX rightMotor = setupWPI_TalonSRX(4, Sides.RIGHT, false);

  TalonSRXSimCollection m_leftDriveSim = leftMotor.getSimCollection();
  TalonSRXSimCollection m_rightDriveSim = rightMotor.getSimCollection();

  AnalogGyro gyro = new AnalogGyro(1);
  AnalogGyroSim m_gyroSim = new AnalogGyroSim(gyro);

  Field2d m_fieldSim  = new Field2d();

  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  // Create the simulation model of our drivetrain.
  DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(
    DCMotor.getCIM(2),       // 2 NEO motors on each side of the drivetrain.
    10.71,                    // 7.29:1 gearing reduction.
    7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
    60.0,                    // The mass of the robot is 60 kg.
    Units.inchesToMeters(4), // The robot uses 4" radius wheels.
    0.69,                  // The track width is 0.7112 meters.

    // The standard deviations for measurement noise:
    // x and y:          0.001 m
    // heading:          0.001 rad
    // l and r velocity: 0.1   m/s
    // l and r position: 0.005 m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  String data = "";
  
  int counter = 0;
  double startTime = 0;
  double priorAutospeed = 0;

  double[] numberArray = new double[10];
  ArrayList<Double> entries = new ArrayList<Double>();
  public Robot() {
    super(.005);
    LiveWindow.disableAllTelemetry();
  }

  public enum Sides {
    LEFT,
    RIGHT,
    FOLLOWER
  }

  // methods to create and setup motors (reduce redundancy)
  public WPI_TalonSRX setupWPI_TalonSRX(int port, Sides side, boolean inverted) {
    // create new motor and set neutral modes (if needed)
    WPI_TalonSRX motor = new WPI_TalonSRX(port);
    // setup talon
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(inverted);
    
    // setup encoder if motor isn't a follower
    if (side != Sides.FOLLOWER) {
    
      
      motor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder,
            PIDIDX, 10
      );    



    switch (side) {
      // setup encoder and data collecting methods

      case RIGHT:
        // set right side methods = encoder methods

          
        motor.setSensorPhase(true);
        rightEncoderPosition = ()
          -> motor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
        rightEncoderRate = ()
          -> motor.getSelectedSensorVelocity(PIDIDX) * encoderConstant *
               10;


        break;
      case LEFT:
        motor.setSensorPhase(false);
        
        leftEncoderPosition = ()
          -> motor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
        leftEncoderRate = ()
          -> motor.getSelectedSensorVelocity(PIDIDX) * encoderConstant *
               10;
        

        break;
      default:
        // probably do nothing
        break;

      }
    
    }
    

    return motor;

  }
  public WPI_VictorSPX setupWPI_VictorSPX(int port, Sides side, boolean inverted) {
    // create new motor and set neutral modes (if needed)
    WPI_VictorSPX motor = new WPI_VictorSPX(port);
    // setup talon
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(inverted);
    
    // setup encoder if motor isn't a follower
    if (side != Sides.FOLLOWER) {
    
      
      motor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder,
            PIDIDX, 10
      );    



    switch (side) {
      // setup encoder and data collecting methods

      case RIGHT:
        // set right side methods = encoder methods

          
        motor.setSensorPhase(false);
        rightEncoderPosition = ()
          -> motor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
        rightEncoderRate = ()
          -> motor.getSelectedSensorVelocity(PIDIDX) * encoderConstant *
               10;


        break;
      case LEFT:
        motor.setSensorPhase(true);
        
        leftEncoderPosition = ()
          -> motor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
        leftEncoderRate = ()
          -> motor.getSelectedSensorVelocity(PIDIDX) * encoderConstant *
               10;
        

        break;
      default:
        // probably do nothing
        break;

      }
    
    }
    

    return motor;

  }

  @Override
  public void robotInit() {
    if (!isReal()) SmartDashboard.putData(new SimEnabler());

    stick = new Joystick(0);

    WPI_VictorSPX leftFollowerID1 = setupWPI_VictorSPX(1, Sides.FOLLOWER, false);
    leftFollowerID1.follow(leftMotor);

    WPI_TalonSRX rightFollowerID3 = setupWPI_TalonSRX(3, Sides.FOLLOWER, false);    
    rightFollowerID3.follow(rightMotor);
    drive = new DifferentialDrive(leftMotor, rightMotor);
    drive.setDeadband(0);

    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of WPILib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    
    gyroAngleRadians = () -> -1 * Math.toRadians(gyro.getAngle());

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  @Override
  public void disabledInit() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Robot disabled");
    drive.tankDrive(0, 0);
    // data processing step
    data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    entries.clear();
    System.out.println("Robot disabled");
    System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
    data = "";
  }

  @Override
  public void simulationPeriodic() {
    m_drivetrainSimulator.setInputs(leftMotor.getMotorOutputVoltage(),
          rightMotor.getMotorOutputVoltage());
    m_drivetrainSimulator.update(0.020);

    m_leftDriveSim.setQuadratureRawPosition(metersToSteps(m_drivetrainSimulator.getLeftPositionMeters()));
    m_leftDriveSim.setQuadratureVelocity(metersPerSecToStepsPerDecisec(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
    m_rightDriveSim.setQuadratureRawPosition(metersToSteps(m_drivetrainSimulator.getRightPositionMeters()));
    m_rightDriveSim.setQuadratureVelocity(metersPerSecToStepsPerDecisec(m_drivetrainSimulator.getRightVelocityMetersPerSecond()));

    m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void robotPeriodic() {
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      leftMotor.getSelectedSensorPosition() * 1.1504855909142309E-4,
      rightMotor.getSelectedSensorPosition() * 1.1504855909142309E-4);
    SmartDashboard.putString("Pose", m_odometry.getPoseMeters().toString());

    // feedback for users, but not used by the control program
    SmartDashboard.putNumber("l_encoder_pos", leftEncoderPosition.get());
    SmartDashboard.putNumber("l_encoder_rate", leftEncoderRate.get());
    SmartDashboard.putNumber("r_encoder_pos", rightEncoderPosition.get());
    SmartDashboard.putNumber("r_encoder_rate", rightEncoderRate.get());
  }

  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");
  }

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-stick.getY(), stick.getX());
  }

  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
    startTime = Timer.getFPGATimestamp();
    counter = 0;
  }

  public double getHeading() {
    return -m_gyroSim.getAngle();
  }

  public static int metersToSteps(double meters) {
    return (int)(meters / Units.inchesToMeters(8) * Math.PI * ENCODER_EDGES_PER_REV);
  }

  public static int metersPerSecToStepsPerDecisec(double metersPerSec) {
    return (int)(metersToSteps(metersPerSec) * .1d);
  }

  /**
  * If you wish to just use your own robot program to use with the data logging
  * program, you only need to copy/paste the logic below into your code and
  * ensure it gets called periodically in autonomous mode
  * 
  * Additionally, you need to set NetworkTables update rate to 10ms using the
  * setUpdateRate call.
  */
  @Override
  public void autonomousPeriodic() {

    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition = leftEncoderPosition.get();
    double leftRate = leftEncoderRate.get();

    double rightPosition = rightEncoderPosition.get();
    double rightRate = rightEncoderRate.get();

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    drive.tankDrive(
      (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
      false
    );

    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();

    // Add data to a string that is uploaded to NT
    for (double num : numberArray) {
      entries.add(num);
    }
    counter++;
  }
}
