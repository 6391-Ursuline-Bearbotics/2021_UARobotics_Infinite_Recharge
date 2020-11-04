package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase implements Loggable{
    @Config
    private final WPI_VictorSPX m_ConveyorMotor1 = new WPI_VictorSPX(ConveyorConstants.kConveyor1ControllerPort);
    @Config
    private final WPI_VictorSPX m_ConveyorMotor2 = new WPI_VictorSPX(ConveyorConstants.kConveyor2ControllerPort);

    AnalogInput frontconveyor = new AnalogInput(2);
    AnalogInput topconveyor = new AnalogInput(3);

    public ConveyorSubsystem() {
        frontconveyor.setAverageBits(4);
        topconveyor.setAverageBits(4);
    }

    @Config
    public void turnOff() {
        this.m_ConveyorMotor1.set(0);
        this.m_ConveyorMotor2.set(0);
    }

    @Config
    public void turnOn() {
        this.m_ConveyorMotor1.set(ConveyorConstants.kConveyorTopMotorSpeed);
        this.m_ConveyorMotor2.set(ConveyorConstants.kConveyorBottomMotorSpeed);
    }

    @Config
    public void turnBackwards() {
        this.m_ConveyorMotor1.set(ConveyorConstants.kConveyorBackSpeed);
        this.m_ConveyorMotor2.set(ConveyorConstants.kConveyorBackSpeed);
    }

    @Config.ToggleButton
    public void toggleConveyor(boolean enabled) {
        if(this.m_ConveyorMotor1.get() > 0) {
            this.m_ConveyorMotor1.set(0);
            this.m_ConveyorMotor2.set(0);
        }
        else{
            this.m_ConveyorMotor1.set(ConveyorConstants.kConveyorTopMotorSpeed);
            this.m_ConveyorMotor2.set(ConveyorConstants.kConveyorBottomMotorSpeed);
        }   
    }

    @Log
    @Log(tabName = "Dashboard", name = "Front Sensor")
    public boolean getFrontConveyor() {
        return (frontconveyor.getAverageVoltage() < 4.75);
    }

    @Log
    @Log(tabName = "Dashboard", name = "Top Sensor")
    public boolean getTopConveyor() {
        return (topconveyor.getAverageVoltage() < 4.75);
    }

    public double getTopVolts() {
        return topconveyor.getAverageVoltage();
    }
}