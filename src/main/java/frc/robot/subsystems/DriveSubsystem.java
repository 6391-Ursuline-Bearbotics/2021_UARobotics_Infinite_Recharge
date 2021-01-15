package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

import java.nio.file.Paths;
import java.io.IOException;

import frc.robot.Constants.DriveConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sim.PhysicsSim;

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

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  
  // The Field2d class simulates the field in the sim GUI. Note that we can have only one
  // instance!
  private Field2d m_fieldSim;

  private AnalogGyroSim m_gyroSim;

  /** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _targetAngle = 0;
  int _printCount = 0;
  private Pose2d savedPose;
  private Trajectory straightTrajectory;
  @Log
  double target_sensorUnits;

  // Turn PIDControllers
  Constraints turnconstraints = new TrapezoidProfile.Constraints(DriveConstants.kMaxSpeedMetersPerSecond,
                                             DriveConstants.kMaxAccelerationMetersPerSecondSquared);
  @Config
  ProfiledPIDController turnangle = new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD, turnconstraints);

  Constraints velocityconstraints = new TrapezoidProfile.Constraints(DriveConstants.kVelocityMaxSpeedMetersPerSecond,
                                             DriveConstants.kVelocityMaxAccelerationMetersPerSecondSquared);
  @Config
  ProfiledPIDController velocityleft = new ProfiledPIDController(DriveConstants.kVelocityP, DriveConstants.kVelocityI, DriveConstants.kVelocityD, velocityconstraints);

  @Config
  ProfiledPIDController velocityright = new ProfiledPIDController(DriveConstants.kVelocityP, DriveConstants.kVelocityI, DriveConstants.kVelocityD, turnconstraints);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Simulation Setup
    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator = new DifferentialDrivetrainSim(
          DriveConstants.kDrivetrainPlant,
          DriveConstants.kDriveGearbox,
          DriveConstants.kDriveGearing,
          DriveConstants.TRACK_WIDTH_METERS,
          DriveConstants.kWheelDiameterMeters / 2.0,
          VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      m_gyroSim = new AnalogGyroSim(1);

      PhysicsSim.getInstance().addTalonSRX(m_talonsrxleft, 0.75, 5100, false);
      PhysicsSim.getInstance().addTalonSRX(m_talonsrxright, 0.75, 5100, false);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();

      SmartDashboard.putData("Field", m_fieldSim);
    }

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    // Set factory defaults
    m_talonsrxleft.configFactoryDefault();
    m_victorspxleft.configFactoryDefault();
    m_talonsrxright.configFactoryDefault();
    m_talonsrxright2.configFactoryDefault();

			// Set followers
    m_victorspxleft.follow(m_talonsrxleft);
		m_talonsrxright2.follow(m_talonsrxright);

    /* Configure output and sensor direction */
		m_talonsrxleft.setInverted(false);
		m_talonsrxleft.setSensorPhase(false);
		m_talonsrxright.setInverted(true);
    m_talonsrxright.setSensorPhase(true);
    m_talonsrxright2.setInverted(true);

    new TalonDriveConfig(m_talonsrxleft, m_talonsrxright, m_pigeon);

    /* Initialize */
    _firstCall = true;
    _state = false;
    _printCount = 0;
    zeroHeading(true);
  }

  public void resetOdometry(Pose2d pose) {
    setCurrentPose(pose);
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
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
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSimulator.setInputs(m_talonsrxleft.getMotorOutputPercent() * RobotController.getBatteryVoltage(),
          m_talonsrxright.getMotorOutputPercent() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);

    m_talonsrxleft.getSimCollection().setQuadratureRawPosition((int)metersToSteps(m_drivetrainSimulator.getLeftPositionMeters()));
    m_talonsrxleft.getSimCollection().setQuadratureVelocity((int)metersPerSecToStepsPerDecisec(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
    m_talonsrxright.getSimCollection().setQuadratureRawPosition((int)metersToSteps(m_drivetrainSimulator.getRightPositionMeters()));
    m_talonsrxright.getSimCollection().setQuadratureVelocity((int)metersPerSecToStepsPerDecisec(m_drivetrainSimulator.getRightVelocityMetersPerSecond()));

    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    m_fieldSim.setRobotPose(getCurrentPose());

    PhysicsSim.getInstance().run();
  }

  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
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
  @Config
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
    if (!RobotBase.isSimulation()) {
      return (m_talonsrxleft.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse
        + m_talonsrxright.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse) / 2.0;
    }
    else {
      return 0;
    }
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
    if (RobotBase.isSimulation()) {
      m_gyroSim.setAngle(0);
    }
    else {
      m_pigeon.setYaw(0, DriveConstants.kTimeoutMs);
      m_pigeon.setAccumZAngle(0, DriveConstants.kTimeoutMs);
      System.out.println("[Pigeon] All sensors are zeroed.\n");
    }
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
    if (RobotBase.isSimulation()) { // If our robot is simulated
      return -m_gyroSim.getAngle();
    }
    else {
      final double[] ypr = new double[3];
		  m_pigeon.getYawPitchRoll(ypr);
      return Math.IEEEremainder(ypr[0], 360);
    }
  }

  @Log
  public double getTurnRate() {
    if (RobotBase.isSimulation()) {
      return m_gyroSim.getRate();
    }
    else {
      final double[] xyz = new double[3];
      m_pigeon.getRawGyro(xyz);
      return xyz[0];
    }
  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftAccel = (leftVelocity - stepsPerDecisecToMetersPerSec((int)m_talonsrxleft.getSelectedSensorVelocity())) / 20;
    var rightAccel = (rightVelocity - stepsPerDecisecToMetersPerSec((int)m_talonsrxright.getSelectedSensorVelocity())) / 20;
    
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
    velocitysetup();
    RamseteCommand ramseteCommand =  new RamseteCommand(
            trajectory,
            this::getCurrentPose,
            new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA),
            DriveConstants.kDriveKinematics,
            this::tankDriveVelocity,
            this);
    this.resetOdometry(straightTrajectory.getInitialPose());
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
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
   * Converts from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }

  public WPI_TalonSRX getRightMaster() {
    return m_talonsrxright;
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

  // Turns to a specified angle using the cascading PID
  public void turnToAngle(double angle) {
    Double angleVelocity = turnangle.calculate(getHeading(), angle);
    tankDrive(velocityleft.calculate(m_talonsrxleft.getSelectedSensorVelocity(), -angleVelocity), velocityright.calculate(m_talonsrxright.getSelectedSensorVelocity(), angleVelocity));
  }

  // Turns to an angle relative to the current angle
  public void turnToRelativeAngle(double angle) {
    angle = 0;
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

    m_talonsrxleft.selectProfileSlot(DriveConstants.kSlot_Velocit, DriveConstants.PID_PRIMARY);
    m_talonsrxleft.selectProfileSlot(DriveConstants.kSlot_Turning, DriveConstants.PID_TURN);
  }

  @Log
  public double getrighterror() {
    //return metersToSteps(m_talonsrxright.getClosedLoopError());
    return m_talonsrxright.getClosedLoopError();
  }

  @Log
  public double getlefterror() {
    return m_talonsrxleft.getClosedLoopError();
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

  @Config
  public void drivePositionGyro(double distanceInches, double heading) {
    var sensorposition = heading * 10;
    distancesetup();
    target_sensorUnits = (distanceInches * DriveConstants.SENSOR_UNITS_PER_ROTATION) / DriveConstants.WHEEL_CIRCUMFERENCE_INCHES ;
    m_talonsrxright.set(ControlMode.Position, target_sensorUnits, DemandType.AuxPID, sensorposition);
  }

  @Config.ToggleButton
  public void drivePositionGyroTest(boolean enabled) {
    new RunCommand(() -> drivePositionGyro(120, getHeading())).withInterrupt(() -> atSetpoint()).withTimeout(5).schedule();
  }

  @Config.ToggleButton
  public void driveVelocityTest(boolean enabled) {
    velocitysetup();
    new RunCommand(() -> tankDriveVelocity(1, -1)).withTimeout(5).schedule();
  }
}