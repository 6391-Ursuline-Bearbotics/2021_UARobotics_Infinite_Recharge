package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Config;

/**
 * A command that will turn the robot to the specified angle.
 */
public class TurnToAngle extends PIDCommand implements Loggable{
  private final DriveSubsystem drive;

  @Config
  private final PIDController controller;

  @Log
  private double logoutput;
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */
  public TurnToAngle(double targetAngleDegrees, DriveSubsystem driveIn) {
    super(
        new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
        // Close loop on heading
        () -> -driveIn.getHeading(),
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> {
          if (output > 0) {
              driveIn.arcadeDrive(0, output + DriveConstants.kTurnFriction);
          } else if (output < 0) {
              driveIn.arcadeDrive(0, output - DriveConstants.kTurnFriction);
          } else {
              driveIn.arcadeDrive(0, output);
          }
        },
        // Require the drive
        driveIn);
    drive = driveIn;
    controller = getController();
    logoutput = controller.calculate(drive.getHeading());
    // Set the controller to be continuous (because it is an angle controller)
    controller.enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    controller.setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  @Log
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return controller.atSetpoint();
  }

  @Log
  public double getPositionError() {
    return controller.getPositionError();
  }

  @Log
  public double getVelocityError() {
    return controller.getVelocityError();
  }
}