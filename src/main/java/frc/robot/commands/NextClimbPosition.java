package frc.robot.commands;

import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NextClimbPosition extends CommandBase{
  private final ClimbSubsystem m_climb;
  /**
   * Creates a new AutoAimCommand.
   * @param m_climb The subsystem used by this command.
   */
  public NextClimbPosition(ClimbSubsystem climb) {
    m_climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.nextClimbStage(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    // End when the climber is at specified position.
    return m_climb.atposition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}