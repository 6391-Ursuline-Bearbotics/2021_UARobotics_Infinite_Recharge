// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.UA6391;

import edu.wpi.first.wpilibj.MotorSafety;

/** Common base class for drive platforms. */
public abstract class RobotDriveBase6391 extends MotorSafety {
  public static final double kDefaultDeadbandForward = 0.02;
  public static final double kDefaultDeadbandRotation = 0.02;
  public static final double kDefaultMaxOutputForward = 1.0;
  public static final double kDefaultMaxOutputRotation = 0.75;
  public static final double kDefaultMinOutputForward = 0;
  public static final double kDefaultMinOutputRotation = 0.2;
  public static final double kDriveStraightLeft = 1;
  public static final double kDriveStraightRight = 1;

  protected double m_deadbandForward = kDefaultDeadbandForward;
  protected double m_deadbandRotation = kDefaultDeadbandRotation;
  protected double m_maxOutputForward = kDefaultMaxOutputForward;
  protected double m_maxOutputRotation = kDefaultMaxOutputRotation;
  protected double m_minOutputForward = kDefaultMaxOutputForward;
  protected double m_minOutputRotation = kDefaultMaxOutputRotation;
  protected double m_driveStraightLeft = kDriveStraightLeft;
  protected double m_driveStraightRight = kDriveStraightRight;

  /** The location of a motor on the robot for the purpose of driving. */
  public enum MotorType {
    kFrontLeft(0),
    kFrontRight(1),
    kRearLeft(2),
    kRearRight(3),
    kLeft(0),
    kRight(1),
    kBack(2);

    public final int value;

    MotorType(int value) {
      this.value = value;
    }
  }

  /** RobotDriveBase constructor. */
  public RobotDriveBase6391() {
    setSafetyEnabled(true);
  }

  /**
   * Sets the deadband applied to the drive inputs (e.g., joystick values).
   *
   * <p>The default value is {@value #kDefaultDeadband}. Inputs smaller than the deadband are set to
   * 0.0 while inputs larger than the deadband are scaled from 0.0 to 1.0. See {@link
   * #applyDeadband}.
   *
   * @param deadband The deadband to set.
   */
  public void setDeadband(double deadbandForward, double deadbandRotation) {
    m_deadbandForward = deadbandForward;
    m_deadbandRotation = deadbandRotation;
  }

  /**
   * Configure the scaling factor for using drive methods with motor controllers in a mode other
   * than PercentVbus or to limit the maximum output.
   *
   * <p>The default value is {@value #kDefaultMaxOutput}.
   *
   * @param maxOutput Multiplied with the output percentage computed by the drive functions.
   */
  public void setMaxOutput(double maxOutputForward, double maxOutputRotation) {
    m_maxOutputForward = maxOutputForward;
    m_maxOutputRotation = maxOutputRotation;
  }

  /**
   * Configure the minimum output.
   *
   * <p>The default value is {@value #kDefaultMaxOutput}.
   *
   * @param maxOutput Multiplied with the output percentage computed by the drive functions.
   */
  public void setMinOutput(double minOutputForward, double minOutputRotation) {
    m_minOutputForward = minOutputForward;
    m_minOutputRotation = minOutputRotation;
  }

  // Sets a number each motor power is multiplied by right before it is set to help the robot drive straight
  public void setDriveStraight(double driveStraightLeft, double driveStraightRight) {
    m_driveStraightLeft = driveStraightLeft;
    m_driveStraightRight = driveStraightRight;
  }

  /**
   * Feed the motor safety object. Resets the timer that will stop the motors if it completes.
   *
   * @see MotorSafety#feed()
   */
  public void feedWatchdog() {
    feed();
  }

  @Override
  public abstract void stopMotor();

  @Override
  public abstract String getDescription();

  /**
   * Returns 0.0 if the given value is within the specified range around zero. The remaining range
   * between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value value to clip
   * @param deadband range around zero
   */
  protected double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /** Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0. */
  protected void normalize(double[] wheelSpeeds) {
    double maxMagnitude = Math.abs(wheelSpeeds[0]);
    for (int i = 1; i < wheelSpeeds.length; i++) {
      double temp = Math.abs(wheelSpeeds[i]);
      if (maxMagnitude < temp) {
        maxMagnitude = temp;
      }
    }
    if (maxMagnitude > 1.0) {
      for (int i = 0; i < wheelSpeeds.length; i++) {
        wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
      }
    }
  }
}
