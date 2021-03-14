package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase implements Loggable{
    private final WPI_VictorSPX m_ConveyorMotor1 = new WPI_VictorSPX(ConveyorConstants.kConveyor1ControllerPort);
    private final WPI_VictorSPX m_ConveyorMotor2 = new WPI_VictorSPX(ConveyorConstants.kConveyor2ControllerPort);

    PowerDistributionPanel m_PDP = new PowerDistributionPanel(0);

    AnalogInput frontconveyor = new AnalogInput(2);
    AnalogInput topconveyor = new AnalogInput(3);

    // The averaging here is so that supurious noise spikes don't trip the sensors.
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
        SmartDashboard.putNumber("FrontSensor", m_PDP.getVoltage());
        return (frontconveyor.getAverageVoltage() < 0.5 && m_PDP.getVoltage() > 11);
    }

    @Log
    @Log(tabName = "Dashboard", name = "Top Sensor")
    public boolean getTopConveyor() {
        SmartDashboard.putNumber("TopSensor", topconveyor.getAverageVoltage());
        return (topconveyor.getAverageVoltage() < 4.75);
    }

    @Log
    public double getTopVolts() {
        return topconveyor.getAverageVoltage();
    }
}