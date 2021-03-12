package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ShooterSubsystem extends PIDSubsystem implements Loggable {
  @Log
  private final WPI_VictorSPX m_shooterMotor = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort2);

  private final WPI_VictorSPX m_shooterMotor2 = new WPI_VictorSPX(ShooterConstants.kShooterMotorPort);

  private final Encoder m_shooterEncoder = new Encoder(ShooterConstants.kEncoderPorts[0],
      ShooterConstants.kEncoderPorts[1], ShooterConstants.kEncoderReversed, CounterBase.EncodingType.k1X);

  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.kSVolts,
      ShooterConstants.kVVoltSecondsPerRotation, ShooterConstants.kA);

  private final PIDController shooterPID;

  @Log
  private double output;

  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  LinearSystem<N2, N1, N1> m_flywheelPosition = LinearSystemId.identifyPositionSystem(ShooterConstants.kVVoltSecondsPerRotation, ShooterConstants.kA);
  LinearSystemSim<N2, N1, N1> m_flywheelPositionSim = new LinearSystemSim<>(m_flywheelPosition);

  //private final DCMotor m_flywheelGearbox = DCMotor.getVex775Pro(2);
  //private final FlywheelSim m_flywheelSim = new FlywheelSim(m_flywheelPositionSim, m_flywheelGearbox, 2);
  private final EncoderSim m_encoderSim = new EncoderSim(m_shooterEncoder);

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
    shooterPID.setTolerance(ShooterConstants.kShooterToleranceRPS, ShooterConstants.kShooterToleranceAccel);
    m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    m_shooterEncoder.setSamplesToAverage(1);
    m_shooterMotor2.follow(m_shooterMotor);
    m_shooterMotor.setInverted(false);
    m_shooterMotor2.setInverted(InvertType.OpposeMaster);
    m_shooterMotor.setNeutralMode(NeutralMode.Coast);
		m_shooterMotor2.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_flywheelPositionSim.setInput(m_shooterMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_flywheelPositionSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_flywheelPositionSim.getOutput(0));
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelPositionSim.getCurrentDrawAmps()));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    var feedforward = m_shooterFeedforward.calculate(setpoint);
    SmartDashboard.putNumber("ShooterPID", output);
    SmartDashboard.putNumber("feedforward", feedforward);
    if (output > 8 && getMeasurement() > 50) {
      // output is still high after being up to speed encoder must be broken
      output = 0;
    }
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
    return shooterPID.atSetpoint() && shooterPID.getSetpoint() > 0 && this.isEnabled();
  }
}