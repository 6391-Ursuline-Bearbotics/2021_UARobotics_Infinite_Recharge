/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// WPI Imports
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
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.commands.AutoAim;
// Command Imports
import frc.robot.commands.Barrel;
import frc.robot.commands.GalacticSearchAuto;
import frc.robot.commands.GenericAuto;
import frc.robot.commands.LStoCP;
import frc.robot.commands.NextClimbPosition;
import frc.robot.commands.Slalom;
import frc.robot.commands.TrenchAuto;
import frc.robot.commands.CenterAuto;
import frc.robot.commands.StealAuto;
import frc.robot.commands.Bounce;
import frc.robot.commands.CPtoLS;
import frc.robot.commands.Center5Ball;
// Subsystem Imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.PhotonVision;
// Constant Imports
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
// Special Imports
import frc.robot.UA6391.Xbox6391;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final PhotonVision m_PhotonVision = new PhotonVision();
  
  @Log
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  @Log
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
 /* 
  private final ControlPanelSubsystem m_controlpanel = new ControlPanelSubsystem(); */
  @Log
  private final LEDSubsystem m_LED = new LEDSubsystem();
  @Log
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  @Log
  public final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
  @Log
  public final ClimbSubsystem m_climb = ClimbSubsystem.Create();
  
  @Log(tabName = "Dashboard")
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

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.getInstance().silenceJoystickConnectionWarning(OIConstants.kPractice);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_robotDrive
            .arcadeDrive(-drv.JoystickLY(), drv.JoystickRX()), m_robotDrive));

    // Constantly checks to see if the intake motor has stalled
    m_intake.setDefaultCommand(new RunCommand(m_intake::checkStall, m_intake));

    m_climb.setDefaultCommand(
      // Use right y axis to control the speed of the climber
      new RunCommand(
        () -> m_climb
          .setOutput(Math.max(op.TriggerL(), drv.TriggerL()),
            Math.max(op.TriggerR(), drv.TriggerR())), m_climb));

    autoChooser.addOption("GalacticSearch", new GalacticSearchAuto(m_robotDrive, m_intake, m_PhotonVision));
    autoChooser.addOption("2R", new GenericAuto(m_robotDrive, "GalacticSearch2R"));
    autoChooser.addOption("Slalom", new Slalom(m_robotDrive));
    autoChooser.addOption("Bounce", new Bounce(m_robotDrive));
    autoChooser.addOption("Barrel", new Barrel(m_robotDrive));
    autoChooser.addOption("Trench Auto", new TrenchAuto(m_shooter, m_robotDrive, m_intake, m_conveyor, m_PhotonVision));
    autoChooser.addOption("Center Auto", new CenterAuto(m_shooter, m_robotDrive, m_intake, m_conveyor, m_PhotonVision));
    autoChooser.addOption("Center 5 Ball", new Center5Ball(m_shooter, m_robotDrive, m_intake, m_conveyor, m_PhotonVision));
    autoChooser.addOption("Steal Auto", new StealAuto(m_shooter, m_robotDrive, m_intake, m_conveyor, m_PhotonVision));
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
        m_shooter.setSetpoint(ShooterConstants.kShooterFarTrenchRPS);
        m_shooter.enable();
      }, m_shooter)))));

    // Stop the Shooter when the B button is pressed
    drv.BButton.or(op.BButton)
      .whenActive(new InstantCommand(() -> {
        m_shooter.setSetpoint(0);
        m_shooter.disable();
      }, m_shooter));
    
    // While driver holds the Y button Auto Aim to the goal using the left stick for distance control
    drv.YButton.whileActiveOnce(new AutoAim(m_robotDrive, m_PhotonVision, m_shooter, () -> -drv.JoystickLY()));

    // When Y button is pressed on operators controller deploy the intake but do not spin the wheels
    op.YButton.whenActive(new InstantCommand(() -> m_intake.toggleIntakePosition(true)));

    // Turn on the conveyor when:
    // the A button is pressed (either controller) and either the top sensor is not blocked or the shooter is up to speed
    // if the bottom sensor is blocked (ball waiting to go up) unless top sensor blocked (the ball has no place to go)
    (topConveyorSensor.negate()
      .and(frontConveyorSensor))
    .or(drv.AButton.and(shooteratsetpoint.or(topConveyorSensor.negate())))
    .or(op.AButton.and(shooteratsetpoint.or(topConveyorSensor.negate())))
    .whenActive(new InstantCommand(m_conveyor::turnOn, m_conveyor))
    .whenInactive(new InstantCommand(m_conveyor::turnOff, m_conveyor));

    // When right bumper is pressed raise/lower the intake and stop/start the intake on both controllers
    drv.BumperR.or(op.BumperR).whenActive(new InstantCommand(() -> m_intake.toggleIntakeWheels(true))
      .andThen(new InstantCommand(() -> m_intake.toggleIntakePosition(true))));
    
    // When the left bumper is pressed on either controller right joystick is super slow turn
    drv.BumperL.or(op.BumperL).whileActiveOnce(new InstantCommand(() -> m_robotDrive.setMaxDriveOutput(
        DriveConstants.kMaxOutputForward, DriveConstants.kMaxOutputRotationSlow)))
      .whenInactive(new InstantCommand(() -> m_robotDrive.setMaxDriveOutput(
        DriveConstants.kMaxOutputForward, DriveConstants.kMaxOutputRotation)));
    //drv.BumperL.whileActiveOnce(m_robotDrive.driveStraight(() -> -drv.JoystickLY()));

    // When the back button is pressed run the conveyor backwards until released
    drv.BackButton.whenActive(new InstantCommand(m_conveyor::turnBackwards, m_conveyor))
      .whenInactive(new InstantCommand(m_conveyor::turnOff, m_conveyor));
    
    // When start button is pressed for at least a second advance to the next climb stage
    drv.StartButton.or(op.StartButton).whileActiveOnce(new WaitCommand(0.5).andThen(new NextClimbPosition(m_climb).withTimeout(5)));

    // Create "button" from POV Hat in up direction.  Use both of the angles to the left and right also.
    //drv.POVUp.whileActiveOnce(new LStoCP(m_shooter, m_robotDrive, m_intake));
    drv.POVUp.whenActive(() -> m_shooter.setSetpoint(ShooterConstants.kShooter1));
    drv.POVRight.whenActive(() -> m_shooter.setSetpoint(ShooterConstants.kShooter2));
    drv.POVDown.whenActive(() -> m_shooter.setSetpoint(ShooterConstants.kShooter3));
    drv.POVLeft.whenActive(() -> m_shooter.setSetpoint(ShooterConstants.kShooter4));

    // Create "button" from POV Hat in down direction.  Use both of the angles to the left and right also.
    //drv.POVDown.whileActiveOnce(new CPtoLS(m_shooter, m_robotDrive, m_intake));

    // POV Up Direction on Operator Controller relatively increases the current setpoint of the shooter
    op.POVUp.whenActive(new InstantCommand(() -> {m_shooter.setSetpoint(m_shooter.getSetpoint() + 1);}));

    // POV Down Direction on Operator Controller relatively increases the current setpoint of the shooter
    op.POVDown.whenActive(new InstantCommand(() -> {m_shooter.setSetpoint(m_shooter.getSetpoint() - 1);}));
  }

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
