package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import java.nio.file.Paths;
import java.io.IOException;

import frc.robot.Constants.DriveConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase implements Loggable{
	// The motors on the left and right side of the drivetrain
	@Log
  private final WPI_TalonSRX m_talonsrxleft = new WPI_TalonSRX(DriveConstants.kLeftMotor2Port);
  
  private final WPI_VictorSPX m_victorspxleft = new WPI_VictorSPX(DriveConstants.kLeftMotor1Port);
  
	@Log
  private final WPI_TalonSRX m_talonsrxright = new WPI_TalonSRX(DriveConstants.kRightMotor2Port);
  
  private final WPI_TalonSRX m_talonsrxright2 = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);

  // Pigeon is plugged into the second talon on the left side
  private final PigeonIMU m_pigeon = new PigeonIMU(m_talonsrxright2);
	
	// Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Using onboard feedforward since it is more accurate than Talon Feedforward
  private final SimpleMotorFeedforward m_driveFeedforward =
      new SimpleMotorFeedforward(DriveConstants.kS,
                                 DriveConstants.kV,
                                 DriveConstants.kA);

  /** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _targetAngle = 0;
  int _printCount = 0;
  private Pose2d savedPose;
  private Trajectory straightTrajectory;
  @Log
  double target_sensorUnits;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
		m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    // Set factory defaults
    m_talonsrxleft.configFactoryDefault();
    m_victorspxleft.configFactoryDefault();
    m_talonsrxright.configFactoryDefault();
    m_talonsrxright2.configFactoryDefault();

			// Set followers
    m_victorspxleft.follow(m_talonsrxleft);
		m_talonsrxright2.follow(m_talonsrxright);

    /* Disable all motor controllers */
		m_talonsrxright.set(0);
		m_talonsrxleft.set(0);
    
    // Set Ramping
    m_talonsrxleft.configClosedloopRamp(DriveConstants.kClosedRamp);
    m_talonsrxleft.configOpenloopRamp(DriveConstants.kOpenRamp);
    m_talonsrxright.configClosedloopRamp(DriveConstants.kClosedRamp);
    m_talonsrxright.configOpenloopRamp(DriveConstants.kOpenRamp);

		/* Set Neutral Mode */
		m_talonsrxleft.setNeutralMode(NeutralMode.Brake);
		m_talonsrxright.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the Pigeon IMU as a Remote Sensor for the right Talon */
		m_talonsrxright.configRemoteFeedbackFilter(m_pigeon.getDeviceID(),			// Device ID of Source
												RemoteSensorSource.Pigeon_Yaw,	// Remote Feedback Source
												DriveConstants.REMOTE_1,				// Remote number [0, 1]
												DriveConstants.kTimeoutMs);			// Configuration Timeout
		
		/* Configure the Remote Sensor to be the Selected Sensor of the right Talon */
		m_talonsrxright.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1, 	// Set remote sensor to be used directly
													DriveConstants.PID_TURN, 			// PID Slot for Source [0, 1]
													DriveConstants.kTimeoutMs);			// Configuration Timeout
		
		/* Scale the Selected Sensor using a coefficient (Values explained in Constants.java */
		m_talonsrxright.configSelectedFeedbackCoefficient(	DriveConstants.kTurnTravelUnitsPerRotation / DriveConstants.kPigeonUnitsPerRotation,	// Coefficient
                            DriveConstants.PID_TURN, 														// PID Slot of Source
														DriveConstants.kTimeoutMs);														// Configuration Timeout
		/* Configure output and sensor direction */
		m_talonsrxleft.setInverted(false);
		m_talonsrxleft.setSensorPhase(true);
		m_talonsrxright.setInverted(true);
    m_talonsrxright.setSensorPhase(true);
    m_talonsrxright2.setInverted(true);
		
		/* Set status frame periods */
		m_talonsrxright.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, DriveConstants.kTimeoutMs);
		m_talonsrxright.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, DriveConstants.kTimeoutMs);
		m_talonsrxright.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, DriveConstants.kTimeoutMs);
		m_pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, DriveConstants.kTimeoutMs);
		
		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		m_talonsrxleft.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
		m_talonsrxleft.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);
		m_talonsrxright.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
		m_talonsrxright.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);

    /* FPID Gains for distance servo */
		m_talonsrxright.config_kP(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kP, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_kI(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kI, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_kD(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kD, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_kF(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kF, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_IntegralZone(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kIzone, DriveConstants.kTimeoutMs);
		m_talonsrxright.configClosedLoopPeakOutput(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kPeakOutput, DriveConstants.kTimeoutMs);

		/* FPID Gains for turn servo */
		m_talonsrxright.config_kP(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kP, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_kI(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kI, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_kD(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kD, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_kF(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kF, DriveConstants.kTimeoutMs);
		m_talonsrxright.config_IntegralZone(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kIzone, DriveConstants.kTimeoutMs);
		m_talonsrxright.configClosedLoopPeakOutput(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kPeakOutput, DriveConstants.kTimeoutMs);
		m_talonsrxright.configAllowableClosedloopError(DriveConstants.kSlot_Turning, 0, DriveConstants.kTimeoutMs);	
		
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
        final int closedLoopTimeMs = 1;
    m_talonsrxright.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.kTimeoutMs);
    m_talonsrxright.configClosedLoopPeriod(1, closedLoopTimeMs, DriveConstants.kTimeoutMs);

    /*
     * configAuxPIDPolarity(boolean invert, int timeoutMs) false means talon's local
     * output is PID0 + PID1, and other side Talon is PID0 - PID1 true means talon's
     * local output is PID0 - PID1, and other side Talon is PID0 + PID1
     */
    m_talonsrxright.configAuxPIDPolarity(false, DriveConstants.kTimeoutMs);

    /* Initialize */
    _firstCall = true;
    _state = false;
    _printCount = 0;
    zeroHeading(true);
  }

  public void resetOdometry() {
    setCurrentPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Resets the current pose to the specified pose. This this ONLY be called
   * when the robot's position on the field is known, like at the beginnig of
   * a match. This will also reset the saved pose since the old pose could be invalidated.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    resetEncoders();
    savedPose = newPose;
    m_odometry.resetPosition(newPose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      m_talonsrxleft.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse,
      m_talonsrxright.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse);
    SmartDashboard.putString("Pose", m_odometry.getPoseMeters().toString());
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  /* public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_talonsrxleft.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
        m_talonsrxright.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse);
  } */

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    fwd = Deadband(fwd);
    rot = Deadband(rot);
    m_talonsrxleft.set(fwd + rot);
    m_talonsrxright.set(fwd - rot);
  }

   /**
   * Drives the robot using tank controls.
   *
   * @param left the commanded left side drivetrain power
   * @param right the commanded right side drivetrain power
   */
  public void tankDrive(double left, double right) {
    left = Deadband(left);
    right = Deadband(right);
    m_talonsrxleft.set(left);
    m_talonsrxright.set(right);
  }

  /**
   * Attempts to follow the given drive states using offboard PID. THIS IS UNTESTED
   *
   * @param left The left wheel state.
   * @param right The right wheel state.
   */
  public void setDriveStates(TrapezoidProfile.State left, TrapezoidProfile.State right) {
    m_talonsrxleft.set(ControlMode.Position,
                             left.position,
                             DemandType.ArbitraryFeedForward,
                             m_driveFeedforward.calculate(left.velocity));
    m_talonsrxright.set(ControlMode.Position,
                              right.position,
                              DemandType.ArbitraryFeedForward,
                              m_driveFeedforward.calculate(right.velocity));
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_talonsrxleft.setSelectedSensorPosition(0);
    m_talonsrxright.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  @Log
  public double getAverageEncoderDistance() {
    return (m_talonsrxleft.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse
        + m_talonsrxright.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse) / 2.0;
  }

  /** Deadband 15 percent, used on the gamepad */
  double Deadband(final double value) {
    /* Upper deadband */
    if (value >= +0.15)
      return value;

    /* Lower deadband */
    if (value <= -0.15)
      return value;

    /* Outside deadband */
    return 0;
  }

  /** Zero all sensors used. */
  @Config.ToggleButton
  void zeroHeading(boolean enabled) {
    m_pigeon.setYaw(0, DriveConstants.kTimeoutMs);
    m_pigeon.setAccumZAngle(0, DriveConstants.kTimeoutMs);
    System.out.println("[Pigeon] All sensors are zeroed.\n");
  }

  public Pose2d getCurrentPose() {
    return m_odometry.getPoseMeters();
  }

  public void saveCurrentPose() {
    savedPose = getCurrentPose();
  }

  public Pose2d getSavedPose() {
    return savedPose;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  @Log
  //@Log(tabName = "Dashboard", name = "Gyro Heading")
  public double getHeading() {
    final double[] ypr = new double[3];
		m_pigeon.getYawPitchRoll(ypr);
    return Math.IEEEremainder(ypr[0], 360);
  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftAccel = (leftVelocity - stepsPerDecisecToMetersPerSec(m_talonsrxleft.getSelectedSensorVelocity())) / 20;
    var rightAccel = (rightVelocity - stepsPerDecisecToMetersPerSec(m_talonsrxright.getSelectedSensorVelocity())) / 20;
    
    var leftFeedForwardVolts = m_driveFeedforward.calculate(leftVelocity, leftAccel);
    var rightFeedForwardVolts = m_driveFeedforward.calculate(rightVelocity, rightAccel);

    m_talonsrxleft.set(
        ControlMode.Velocity, 
        metersPerSecToStepsPerDecisec(leftVelocity), 
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);
    m_talonsrxright.set(
        ControlMode.Velocity,
        metersPerSecToStepsPerDecisec(rightVelocity),
        DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts / 12);
  }

  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(String trajname) {
    try {
      straightTrajectory = loadTrajectory(trajname);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + trajname, false);
    }
    Transform2d transform = new Pose2d(0, 0, Rotation2d.fromDegrees(0)).minus(straightTrajectory.getInitialPose());
    Trajectory trajectory = straightTrajectory.transformBy(transform);
    return new RamseteCommand(
            trajectory,
            this::getCurrentPose,
            new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA),
            DriveConstants.kDriveKinematics,
            this::tankDriveVelocity,
            this);
  }

    /**
   * Converts from encoder steps to meters.
   * 
   * @param steps encoder steps to convert
   * @return meters
   */
  public static double stepsToMeters(int steps) {
    return (DriveConstants.WHEEL_CIRCUMFERENCE_METERS / DriveConstants.SENSOR_UNITS_PER_ROTATION) * steps;
  }

  /**
   * Converts from encoder units per 100 milliseconds to meters per second.
   * @param stepsPerDecisec steps per decisecond
   * @return meters per second
   */
  public static double stepsPerDecisecToMetersPerSec(int stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10);
  }

  /**
   * Converts from meters to encoder units.
   * @param meters meters
   * @return encoder units
   */
  public static double metersToSteps(double meters) {
    return (meters / DriveConstants.WHEEL_CIRCUMFERENCE_METERS) * DriveConstants.SENSOR_UNITS_PER_ROTATION;
  }

  /**
   * Convers from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }

  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", trajectoryName + ".wpilib.json")));
  }

  // Drives straight specified distance in inches
  public void drivestraight(double distance) {
    target_sensorUnits = (distance * DriveConstants.SENSOR_UNITS_PER_ROTATION) / DriveConstants.WHEEL_CIRCUMFERENCE_INCHES ;
    m_talonsrxright.set(ControlMode.Position, target_sensorUnits, DemandType.AuxPID, m_talonsrxright.getSelectedSensorPosition(1));
		m_talonsrxleft.follow(m_talonsrxright, FollowerType.AuxOutput1);
  }

  // Sets up the talons to drive straightDistance with aux pid from Pigeon 
  public void distancesetup() {
    resetEncoders();
				
    /* Determine which slot affects which PID */
    m_talonsrxright.selectProfileSlot(DriveConstants.kSlot_Distanc, DriveConstants.PID_PRIMARY);
    m_talonsrxright.selectProfileSlot(DriveConstants.kSlot_Turning, DriveConstants.PID_TURN);
    m_talonsrxleft.follow(m_talonsrxright, FollowerType.AuxOutput1);
  }

  public void velocitysetup() {
    resetEncoders();
				
    /* Determine which slot affects which PID */
    m_talonsrxright.selectProfileSlot(DriveConstants.kSlot_Velocit, DriveConstants.PID_PRIMARY);
    m_talonsrxright.selectProfileSlot(DriveConstants.kSlot_Turning, DriveConstants.PID_TURN);
  }

  @Log
  public double getrighterror() {
    return metersToSteps(m_talonsrxright.getClosedLoopError());
  }

  @Log
  public double getlefterror() {
    return metersToSteps(m_talonsrxleft.getClosedLoopError());
  }

  @Log
  public boolean atSetpoint() {
    if (m_talonsrxright.getClosedLoopError() < 1000 && m_talonsrxright.getClosedLoopError() > 0){
      return true;
    } else {
      return false;
    }
  }

  public void stopmotors(boolean enabled) {
    m_talonsrxright.set(0);
    m_talonsrxleft.set(0);
  }

  @Log
  public void tankDriveVolts(final double leftVolts, final double rightVolts) {
    m_talonsrxleft.setVoltage(leftVolts);
    m_talonsrxright.setVoltage(rightVolts);
    //m_drive.feed();
  }

  public Command driveTime(double time, double speed) {
    return new RunCommand(() -> {m_talonsrxleft.set(speed);
      m_talonsrxright.set(speed);})
      .withTimeout(time)
      .andThen(() -> {m_talonsrxleft.set(0);m_talonsrxright.set(0);});
  }

  public Command drivePositionGyro(double distance) {
    target_sensorUnits = (distance * DriveConstants.SENSOR_UNITS_PER_ROTATION) / DriveConstants.WHEEL_CIRCUMFERENCE_INCHES ;
    return new RunCommand(() -> 
    m_talonsrxright.set(ControlMode.Position, target_sensorUnits, DemandType.AuxPID, m_talonsrxright.getSelectedSensorPosition(1)))
    .withInterrupt(() -> atSetpoint());
  }
}