/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// WPI Imports
import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import io.github.oblarg.oblog.annotations.Log;

// Command Imports
import frc.robot.commands.AutoAim;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveDistanceProfiled;
import frc.robot.commands.NextClimbPosition;
import frc.robot.commands.TrenchAuto;
import frc.robot.commands.CenterAuto;
import frc.robot.commands.StealAuto;
// Subsystem Imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Limelight;

// Constant Imports
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.OIConstants;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  @Log
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  @Log
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
 /*  @Log
  private final ControlPanelSubsystem m_controlpanel = new ControlPanelSubsystem(); */
  @Log
  private final LEDSubsystem m_LED = new LEDSubsystem();
  @Log
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  @Log
  public final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
  @Log
  public final ClimbSubsystem m_climb = ClimbSubsystem.Create();

  public final Limelight m_Limelight = new Limelight();

  @Log.PDP
  PowerDistributionPanel m_PDP = new PowerDistributionPanel(0);
  
  // Creating this so we get logging in the Command
  Command DriveStraight = new DriveStraight(0, m_robotDrive);

  @Log(tabName = "DriveSubsystem")
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // The driver's controller
  Xbox6391 drv = new Xbox6391(OIConstants.kDriverControllerPort);
  XboxControllerSim m_driverControllerSim = new XboxControllerSim(OIConstants.kDriverControllerPort);

  // The operator's controller
  Xbox6391 op = new Xbox6391(OIConstants.kOperatorControllerPort);
  XboxControllerSim m_operatorControllerSim = new XboxControllerSim(OIConstants.kOperatorControllerPort);

  Button frontConveyorSensor = new Button(() -> m_conveyor.getFrontConveyor());
  Button topConveyorSensor = new Button(() -> m_conveyor.getTopConveyor());
  Button shooteratsetpoint = new Button(() -> m_shooter.atSetpoint());
  public HttpCamera m_limelightFeed;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.getInstance().silenceJoystickConnectionWarning(OIConstants.kPractice);

    // Configure the button bindings
    configureButtonBindings();

    // Limelight Setup
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
    //LimelightCamera();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_robotDrive
            .arcadeDrive(-drv.JoystickLY(), drv.JoystickRX()), m_robotDrive));

/*     m_climb.setDefaultCommand(
      // Use right y axis to control the speed of the climber
      new RunCommand(
        () -> m_climb
          .setOutput(Math.max(m_operatorController.getRawAxis(2),m_driverController.getRawAxis(2)),
            Math.max(m_operatorController.getRawAxis(3), m_driverController.getRawAxis(3))), m_climb)); */
                         
    // Sets the LEDs to start up with a rainbow config
    //m_LED.rainbow();

    autoChooser.addOption("Auto Aim", new AutoAim(m_robotDrive));
    autoChooser.addOption("Trench Auto", new TrenchAuto(m_shooter, m_robotDrive, m_intake, m_conveyor));
    autoChooser.addOption("Center Auto", new CenterAuto(m_shooter, m_robotDrive, m_intake, m_conveyor));
    autoChooser.addOption("Steal Auto", new StealAuto(m_shooter, m_robotDrive, m_intake, m_conveyor));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Spin up the shooter to far trench speed when the 'X' button is pressed.
    drv.XButton.or(op.XButton)
        .whenActive(new InstantCommand(m_conveyor::turnBackwards)
        .andThen(new WaitCommand(.15)
        .andThen(new InstantCommand(m_conveyor::turnOff)
        .andThen(new InstantCommand(() -> {
        m_shooter.setSetpoint(ShooterConstants.kShooterFarTrenchRPM);
        m_shooter.enable();
      }, m_shooter)))));

    // Stop the Shooter when the B button is pressed
    drv.BButton.or(op.BButton)
      .whenActive(new InstantCommand(() -> {
        m_shooter.setSetpoint(0);
        m_shooter.disable();
      }, m_shooter));
    
    // When driver presses the Y button Auto Aim to the goal
    drv.YButton.whenPressed(new InstantCommand(() -> m_Limelight.beforeTurnToTarget()))
      .whileHeld(new InstantCommand(() -> m_Limelight.turnToTargetVolts(m_robotDrive,m_shooter), m_robotDrive))
      .whenReleased(new InstantCommand(() -> m_Limelight.afterTurnToTarget()));
    //.whenPressed(new AutoAim(m_robotDrive));

    // When Y button is pressed on operators controller deploy the intake but do not spin the wheels
    op.YButton.whenPressed(new InstantCommand(() -> m_intake.toggleIntakePosition(true)));

    // Turn on the conveyor when either the A button is pressed or if the bottom sensor is blocked
    // (new ball) and the top sensor is not blocked (ball has a place to go)
    (topConveyorSensor.negate()
      .and(frontConveyorSensor))
    .or(drv.AButton.and(shooteratsetpoint.or(topConveyorSensor.negate())))
    .or(op.AButton.and(shooteratsetpoint.or(topConveyorSensor.negate())))
    .whenActive(new InstantCommand(m_conveyor::turnOn, m_conveyor))
    .whenInactive(new InstantCommand(m_conveyor::turnOff, m_conveyor));

    // When right bumper is pressed raise/lower the intake and stop/start the intake on both controllers
    drv.BumperR.or(op.BumperR).whenActive(new InstantCommand(() -> m_intake.toggleIntakeWheels(true))
      .andThen(new InstantCommand(() -> m_intake.toggleIntakePosition(true))));
    
    // When the left bumper is pressed on either controller go to the next climber stage
    drv.BumperL.or(op.BumperL).whileActiveOnce(new WaitCommand(1).andThen(new NextClimbPosition(m_climb).withTimeout(5)));
     // new PerpetualCommand(new InstantCommand(() -> m_climb.nextClimbStage(true))
     // .withInterrupt(() -> m_climb.atposition())));
     // .whenActive(new InstantCommand(() -> m_climb.nextClimbStage(true))
     //   .perpetually().withInterrupt(() -> m_climb.atposition()));

    // TEST when back is pressed drive straight 120 inches
    /* new JoystickButton(m_driverController, XboxController.Button.kBack.value)
      .or(new JoystickButton(m_operatorController, XboxController.Button.kBack.value))
      .whenActive(new DriveStraight(120, m_robotDrive).withTimeout(10)); */

    drv.BackButton.whenPressed(() -> m_robotDrive.tankDriveVelocity(1, 1));
    //.whenPressed(new InstantCommand(m_conveyor::turnBackwards, m_conveyor))
    //  .whenReleased(new InstantCommand(m_conveyor::turnOff, m_conveyor));
    
    // TEST when start is pressed follow trajectory
    drv.StartButton.whenPressed(() -> m_robotDrive.createCommandForTrajectory("Straight").withTimeout(5).schedule());
    //.whenPressed(new DriveDistanceProfiled(3, m_robotDrive).withTimeout(10));

    // Create "button" from POV Hat in up direction.  Use both of the angles to the left and right also.
    drv.POVUp.whenActive(new RunCommand(() -> m_robotDrive.turnToAngle(90)).withTimeout(5));
    
    // Create "button" from POV Hat in down direction.  Use both of the angles to the left and right also.
    drv.POVDown.whenActive(new RunCommand(() -> m_robotDrive.turnToAngle(-90)).withTimeout(5));

    // POV Up Direction on Operator Controller relatively increases the current setpoint of the shooter
    op.POVUp.whenActive(new InstantCommand(() -> {m_shooter.setSetpoint(m_shooter.getSetpoint() + 50);}));

    // POV Down Direction on Operator Controller relatively increases the current setpoint of the shooter
    op.POVDown.whenActive(new InstantCommand(() -> {m_shooter.setSetpoint(m_shooter.getSetpoint() - 50);}));
  }
  
/*   public void LimelightCamera() {
    // Activate an HttpCamera for the Limelight
    m_limelightFeed = new HttpCamera("Limelight Camera", "http://10.63.91.11:5800/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer);
  }

  @Log.CameraStream(name = "Limelight Camera", tabName = "Dashboard")
  public HttpCamera getLimelightFeed() {
    return m_limelightFeed;
  } */

  public DriveSubsystem getRobotDrive() {
    return m_robotDrive;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
