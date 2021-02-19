package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import java.nio.file.Paths;
import java.io.IOException;

import frc.robot.UA6391.DifferentialDrive6391;
import frc.robot.UA6391.Trajectory6391;
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

  private final DifferentialDrive6391 m_drive = new DifferentialDrive6391(m_talonsrxleft, m_talonsrxright);

  // Object for simulated inputs into Talon.
  TalonSRXSimCollection m_leftDriveSim = m_talonsrxleft.getSimCollection();
  TalonSRXSimCollection m_rightDriveSim = m_talonsrxright.getSimCollection();

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

  // The pidgeon is currently supported so we create this analog gyro that doesn't exist to replace it.
  private AnalogGyroSim m_gyroSim;

  /** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _targetAngle = 0;
  int _printCount = 0;
  private Pose2d savedPose;
  @Log
  double target_sensorUnits;
  double m_time = 0;
  double m_lastTime = Timer.getFPGATimestamp() - 0.02;
  double m_lastLeftSetpoint = 0;
  double m_lastRightSetpoint = 0;

  // Turn PIDControllers
  Constraints turnconstraints = new TrapezoidProfile.Constraints(DriveConstants.kMaxSpeedMetersPerSecond,
                                             DriveConstants.kMaxAccelerationMetersPerSecondSquared);

  ProfiledPIDController turnangle = new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD, turnconstraints);

  Constraints velocityconstraints = new TrapezoidProfile.Constraints(DriveConstants.kVelocityMaxSpeedMetersPerSecond,
                                             DriveConstants.kVelocityMaxAccelerationMetersPerSecondSquared);

  ProfiledPIDController velocityleft = new ProfiledPIDController(DriveConstants.kVelocityP, DriveConstants.kVelocityI, DriveConstants.kVelocityD, velocityconstraints);

  ProfiledPIDController velocityright = new ProfiledPIDController(DriveConstants.kVelocityP, DriveConstants.kVelocityI, DriveConstants.kVelocityD, turnconstraints);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Differential Drive Configuration
    m_drive.setMaxOutput(DriveConstants.kMaxOutputForward, DriveConstants.kMaxOutputRotation);
    m_drive.setMinOutput(DriveConstants.kMinOutputForward, DriveConstants.kMinOutputRotation);
    m_drive.setDeadband(DriveConstants.kDeadbandForward, DriveConstants.kDeadbandRotation);
    m_drive.setRamp(DriveConstants.kRampForward, DriveConstants.kRampRotation);
    m_drive.setDriveStraight(DriveConstants.kDriveStraightLeft, DriveConstants.kDriveStraightRight);
    m_drive.setRightSideInverted(false);

    // Simulation Setup
    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator = new DifferentialDrivetrainSim(
          DriveConstants.kDrivetrainPlant,
          DriveConstants.kDriveGearbox,
          DriveConstants.kDriveGearing,
          DriveConstants.kTrackWidthMeters,
          DriveConstants.kWheelDiameterMeters / 2.0,
          null);

      // This doesn't exist but Pidgeon isn't supported yet.
      m_gyroSim = new AnalogGyroSim(1);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();

      SmartDashboard.putData("Field", m_fieldSim);
    }

    // Odometry is the object that updates the pose (robots position and angle on the field)
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    // Set factory defaults
    m_talonsrxleft.configFactoryDefault();
    m_victorspxleft.configFactoryDefault();
    m_talonsrxright.configFactoryDefault();
    m_talonsrxright2.configFactoryDefault();

			// Set followers
    m_victorspxleft.follow(m_talonsrxleft);
		m_talonsrxright2.follow(m_talonsrxright);

    // Simulation Setup
    if (RobotBase.isSimulation()) { // If our robot is simulated
      m_talonsrxleft.setInverted(false);
      m_talonsrxleft.setSensorPhase(false);
      m_talonsrxright.setInverted(true);
      m_talonsrxright.setSensorPhase(true);
      m_talonsrxright2.setInverted(true);
    }
    else {
      m_talonsrxleft.setInverted(false);
      m_talonsrxleft.setSensorPhase(true);
      m_talonsrxright.setInverted(true);
      m_talonsrxright.setSensorPhase(true);
      m_talonsrxright2.setInverted(true);
    }

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
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    //m_drivetrainSimulator.setInputs(m_talonsrxleft.getMotorOutputPercent() * RobotController.getBatteryVoltage(),
    //      m_talonsrxright.getMotorOutputPercent() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.setInputs(m_talonsrxleft.getMotorOutputVoltage(),
          m_talonsrxright.getMotorOutputVoltage());
    m_drivetrainSimulator.update(0.020);

    m_leftDriveSim.setQuadratureRawPosition(metersToSteps(m_drivetrainSimulator.getLeftPositionMeters()));
    m_leftDriveSim.setQuadratureVelocity(metersPerSecToStepsPerDecisec(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
    m_rightDriveSim.setQuadratureRawPosition(metersToSteps(m_drivetrainSimulator.getRightPositionMeters()));
    m_rightDriveSim.setQuadratureVelocity(metersPerSecToStepsPerDecisec(m_drivetrainSimulator.getRightVelocityMetersPerSecond()));

    m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    m_fieldSim.setRobotPose(getCurrentPose());
  }

  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  @Log
  public double getLeftPower() {
    return m_talonsrxleft.getMotorOutputPercent();
  }

  @Log
  public double getRightPower() {
    return m_talonsrxright.getMotorOutputPercent();
  }

  /**
   * Returns the current left wheel speed of the robot.
   *
   * @return The current left wheel speed.
   */
  @Log
  public double getLeftWheelSpeed() {
    // Native units per 100ms so * 10 = native per second
    return stepsPerDecisecToMetersPerSec((int)m_talonsrxleft.getSelectedSensorVelocity());
  }

  /**
   * Returns the current right wheel speed of the robot.
   *
   * @return The current right wheel speed.
   */
  @Log
  public double getRightWheelSpeed() {
    // Native units per 100ms so * 10 = native per second
    return stepsPerDecisecToMetersPerSec((int)m_talonsrxright.getSelectedSensorVelocity());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

   /**
   * Drives the robot using tank controls.
   *
   * @param left the commanded left side drivetrain power
   * @param right the commanded right side drivetrain power
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
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
  @Config.ToggleButton
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

  // This is the closed loop velocity control method we use trajectory following.
  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    m_time = Timer.getFPGATimestamp();
    double dt = m_time - m_lastTime;
    var leftAccel = (leftVelocity - m_lastLeftSetpoint) / dt;
    var rightAccel = (rightVelocity - m_lastRightSetpoint) / dt;
    m_lastTime = m_time;
    m_lastLeftSetpoint = leftVelocity;
    m_lastRightSetpoint = rightVelocity;
    
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

    m_drive.feed();

    SmartDashboard.putNumber("left setpoint", leftVelocity);
    SmartDashboard.putNumber("right setpoint", rightVelocity);
  }

  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {
    velocitysetup();
    RamseteCommand ramseteCommand =  new RamseteCommand(
            trajectory,
            this::getCurrentPose,
            new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA),
            DriveConstants.kDriveKinematics,
            this::tankDriveVelocity,
            this);
    if (initPose) {
      var reset =  new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose()));
      return reset.andThen(ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0)));
    }
    else {
      return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
    }
  }

  public Trajectory loadTrajectoryFromFile(String filename) {
    try {
      return loadTrajectory(filename);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
      return new Trajectory();
    }
  }

  public Trajectory generateTrajectoryFromFile(String filename) {
      var config = new TrajectoryConfig(1, 3);
      return generateTrajectory(filename, config);
  }

    /**
   * Converts from encoder steps to meters.
   * 
   * @param steps encoder steps to convert
   * @return meters
   */
  public static double stepsToMeters(int steps) {
    return DriveConstants.kEncoderDistancePerPulse * steps;
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
  public static int metersToSteps(double meters) {
    return (int)(meters / DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Converts from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static int metersPerSecToStepsPerDecisec(double metersPerSec) {
    return (int)(metersToSteps(metersPerSec) * .1d);
  }

  public WPI_TalonSRX getRightMaster() {
    return m_talonsrxright;
  }

  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(Paths.get("output", trajectoryName + ".wpilib.json")));
  }

  public Trajectory generateTrajectory(String trajectoryName, TrajectoryConfig config) {
    try {
      var filepath = Filesystem.getDeployDirectory().toPath().resolve(Paths.get("waypoints", trajectoryName));
      return Trajectory6391.fromWaypoints(filepath, config);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + trajectoryName, false);
      return new Trajectory();
    }
  }

  // Drives straight specified distance in inches
  public void drivestraight(double distance) {
    target_sensorUnits = (distance / DriveConstants.kEncoderDistancePerPulse);
    m_talonsrxright.set(ControlMode.Position, target_sensorUnits, DemandType.AuxPID, m_talonsrxright.getSelectedSensorPosition(1));
		m_talonsrxleft.follow(m_talonsrxright, FollowerType.AuxOutput1);
  }

  // Turns to a specified angle using the cascading PID
  public void turnToAngle(double angle) {
    Double angleVelocity = turnangle.calculate(getHeading(), angle);
    tankDriveVelocity(velocityleft.calculate(m_talonsrxleft.getSelectedSensorVelocity(), -angleVelocity), velocityright.calculate(m_talonsrxright.getSelectedSensorVelocity(), angleVelocity));
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

  // Drives for a specified time at a specified speed.
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
    target_sensorUnits = (distanceInches / DriveConstants.kEncoderDistancePerPulse);
    m_talonsrxright.set(ControlMode.Position, target_sensorUnits, DemandType.AuxPID, sensorposition);
  }

  @Config.ToggleButton
  public void drivePositionGyroTest(boolean enabled) {
    new RunCommand(() -> drivePositionGyro(120, getHeading())).withInterrupt(() -> atSetpoint()).withTimeout(5).schedule();
  }

  @Config.ToggleButton
  public void driveVelocityTest(boolean enabled) {
    velocitysetup();
    new RunCommand(() -> tankDriveVelocity(1, 1)).withTimeout(5).schedule();
  }
}