package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Config;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ShooterSubsystem extends PIDSubsystem implements Loggable{
  @Log
  private final WPI_VictorSPX m_shooterMotor = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort);

  private final WPI_VictorSPX m_shooterMotor2 = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort2);

  private final Encoder m_shooterEncoder =
      new Encoder(ShooterConstants.kEncoderPorts[0], ShooterConstants.kEncoderPorts[1],
                  ShooterConstants.kEncoderReversed, CounterBase.EncodingType.k1X);

  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(ShooterConstants.kSVolts,
                                 ShooterConstants.kVVoltSecondsPerRotation,
                                 ShooterConstants.kA);

  @Config
  private final PIDController shooterPID;

  @Log
  private double output;

  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  private final LinearSystem<N1, N1, N1> m_flywheelPlant = LinearSystemId.identifyVelocitySystem(
    ShooterConstants.kVVoltSecondsPerRotation, ShooterConstants.kA);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
        Nat.N1(), Nat.N1(),
        m_flywheelPlant,
        VecBuilder.fill(3.0), // How accurate we think our model is
        VecBuilder.fill(0.01), // How accurate we think our encoder
        // data is
        0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller
        = new LinearQuadraticRegulator<>(m_flywheelPlant,
        VecBuilder.fill(8.0), // Velocity error tolerance
        VecBuilder.fill(12.0), // Control effort (voltage) tolerance
        0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
      m_flywheelPlant,
        m_controller,
        m_observer,
        12.0,
        0.020);

  private LinearFilter m_velocityFilterMA = LinearFilter.movingAverage(4);
  private LinearFilter m_velocityFilterIIR = LinearFilter.singlePoleIIR(.1, .02);

  private double m_time = 0;
  private double m_lastTime = Timer.getFPGATimestamp() - 0.02;
  private double m_angle = 0;
  private double m_lastAngle = 0;

  private double m_angularVelocity = 0;

  // The shooter subsystem for the robot.
  public ShooterSubsystem() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    shooterPID = getController();
    shooterPID.setTolerance(ShooterConstants.kShooterToleranceRPM);
    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    m_shooterEncoder.setSamplesToAverage(50);
    m_shooterMotor2.follow(m_shooterMotor);
    m_shooterMotor.setInverted(true);
    m_shooterMotor2.setInverted(InvertType.OpposeMaster);
    m_shooterMotor.setNeutralMode(NeutralMode.Coast);
		m_shooterMotor2.setNeutralMode(NeutralMode.Coast);
  }

  public void loopreset() {
    // Reset our loop to make sure it's in a known state.
    m_loop.reset(VecBuilder.fill(m_angularVelocity));
  }

  @Override
  public void simulationPeriodic() {
    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(m_angularVelocity));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    m_shooterMotor.setVoltage(nextVoltage);
  }

  public void nextR(double spinUp) {
    m_loop.setNextR(VecBuilder.fill(spinUp));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    var feedforward = m_shooterFeedforward.calculate(setpoint);
    SmartDashboard.putNumber("ShooterPID", output);
    SmartDashboard.putNumber("feedforward", feedforward);
    m_shooterMotor.setVoltage(MathUtil.clamp(output + feedforward, 0, 14));
  }

  @Log
  @Log(tabName = "Dashboard", name = "Shooter Speed")
  @Override
  public double getMeasurement() {
    // old way was getRate
    m_angle = getAngle();
    m_time = Timer.getFPGATimestamp();
    
    m_angularVelocity = m_velocityFilterMA.calculate((m_angle - m_lastAngle) / (m_time - m_lastTime));
    m_lastTime = m_time;
    m_lastAngle = m_angle;

    m_angularVelocity = m_velocityFilterIIR.calculate(m_angularVelocity);
    return m_angularVelocity;
  }

  public double getAngle() {
    return m_shooterEncoder.getDistance();
  }

  @Log
  @Log(tabName = "Dashboard", name = "Shooter Setpoint")
  public double getSetpoint() {
    return shooterPID.getSetpoint();
  }

  @Log
  @Log(tabName = "Dashboard", name = "Good to Shoot?")
  public boolean atSetpoint() {
    return shooterPID.atSetpoint() && shooterPID.getSetpoint() > 0;
  }
}