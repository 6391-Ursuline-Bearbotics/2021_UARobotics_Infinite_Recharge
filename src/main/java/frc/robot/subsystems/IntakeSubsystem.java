package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.IntakeConstants;
import frc.robot.UA6391.StallDetector;

public class IntakeSubsystem extends SubsystemBase implements Loggable{
    @Config
    private final WPI_VictorSPX m_IntakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeControllerPort);
    @Config
    private final DoubleSolenoid m_intakeSolenoid1 = new DoubleSolenoid(IntakeConstants.kSolenoid1ControllerPort, IntakeConstants.kSolenoid2ControllerPort);
    @Config
    private final DoubleSolenoid m_intakeSolenoid2 = new DoubleSolenoid(IntakeConstants.kSolenoid3ControllerPort, IntakeConstants.kSolenoid4ControllerPort);

    StallDetector intakeStall;
    // Since we are using DoubleSolenoids they must be initially set so when they are toggled they 
    // know which direction to toggle to as their default state is kOff.
    public IntakeSubsystem() {
        setOutput(0);
        m_intakeSolenoid1.set(Value.kReverse);
        m_intakeSolenoid2.set(Value.kReverse);
        intakeStall = new StallDetector(IntakeConstants.kIntakePDPSlot);
        intakeStall.setMinStallMillis(IntakeConstants.kStallTimeMS);
    }

    @Config
    public void setOutput(double speed) {
        this.m_IntakeMotor.set(speed);
    }

    @Config
    public void toggleIntakePosition(boolean enabled) {
        m_intakeSolenoid1.toggle();
        m_intakeSolenoid2.toggle();        
    }

    @Config
    public void toggleIntakeWheels(boolean enabled) {
        // Only turn it on if intake is down and it is currently off
        if(m_IntakeMotor.get() == 0 && m_intakeSolenoid1.get() == DoubleSolenoid.Value.kReverse) {
            m_IntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
        }
        else{
            m_IntakeMotor.set(0);
        }
    }

    public void checkStall() {
        double current = intakeStall.updateStallStatus();
        if (intakeStall.getStallStatus().isStalled) {
            setOutput(0);
        }
        SmartDashboard.putNumber("IntakeCurrent", current);
    }
}