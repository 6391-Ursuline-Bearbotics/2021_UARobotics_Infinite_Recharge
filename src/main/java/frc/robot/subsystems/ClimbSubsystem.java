package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.Constants.ClimbConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ClimbSubsystem extends SubsystemBase implements Loggable{
    @Config(name="ClimbMotorLeft")
    private WPI_TalonSRX m_LeftClimbMotor;
    
    @Config(name="ClimbMotorRight")
    private WPI_TalonSRX m_RightClimbMotor;

    @Log
    private int climbinvert = 1;

    @Log
    public int climbstage = 0;

    @Log
    public int setpoint = 4200;

    public ClimbSubsystem(WPI_TalonSRX m_LeftClimbMotor, WPI_TalonSRX m_RightClimbMotor) {
        this.m_LeftClimbMotor = m_LeftClimbMotor;
        this.m_RightClimbMotor = m_RightClimbMotor;

        setOutput(0,0);
        m_LeftClimbMotor.setInverted(true);
        m_LeftClimbMotor.setSensorPhase(true);
        m_RightClimbMotor.setInverted(false);
        m_RightClimbMotor.setSensorPhase(true);
        m_LeftClimbMotor.config_kP(0, ClimbConstants.kClimbP);
        m_RightClimbMotor.config_kP(0, ClimbConstants.kClimbP);
        m_LeftClimbMotor.configPeakOutputReverse(0);
        m_RightClimbMotor.configPeakOutputReverse(0);
    }

    public static ClimbSubsystem Create() {
        WPI_TalonSRX m_LeftClimbMotor = new WPI_TalonSRX(ClimbConstants.kClimbLeftControllerPort);
        WPI_TalonSRX m_RightClimbMotor = new WPI_TalonSRX(ClimbConstants.kClimbRightControllerPort);
        return new ClimbSubsystem(m_LeftClimbMotor, m_RightClimbMotor);
    }

    // This is the open loop control of the climber when the "triggers" are used to manually control it
    public void setOutput(double leftMotorPercent, double rightMotorPercent) {
        this.m_LeftClimbMotor.set(leftMotorPercent * climbinvert);
        this.m_RightClimbMotor.set(rightMotorPercent * climbinvert);

        // As soon as we start raising the hooks switch the camera so the hook view is larger
        if(leftMotorPercent > .5 || rightMotorPercent > .5) {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);
        }
    }

    // This is the closed loop position control of the climber
    @Config
    public void setPosition(double position) {
        m_RightClimbMotor.set(ControlMode.Position, position);
        m_LeftClimbMotor.set(ControlMode.Position, position);
    }

    @Log
    public double getRightPosition() {
        return m_RightClimbMotor.getSelectedSensorPosition();
    }

    @Log
    public double getLeftPosition() {
        return m_LeftClimbMotor.getSelectedSensorPosition();
    } 

    @Config.ToggleButton
    public void resetEnc(boolean enabled) {
        m_LeftClimbMotor.setSelectedSensorPosition(0);
        m_RightClimbMotor.setSelectedSensorPosition(0);
    }

    // This is very important for resetting the climber.  This should only be done in the pits with extreme care!
    @Config.ToggleButton
    public void invertclimber(boolean enabled) {
        if (enabled) {
            climbinvert = -1;
            m_LeftClimbMotor.configPeakOutputReverse(-1);
            m_RightClimbMotor.configPeakOutputReverse(-1);
        }
        else {
            climbinvert = 1;
            m_LeftClimbMotor.configPeakOutputReverse(0);
            m_RightClimbMotor.configPeakOutputReverse(0);
        }
    }

    // There are 3 "stages" that we go to.  While lining up we go to the first stage which raise
    // the climbers all the way up.  Once we move forward and touch the bar we bring them down
    // to the point they are on the bar but not actually pulling allowing teammates to get on but
    // it they were to pull ahead of time we would raise too.  Then the actual hang.
    @Config.ToggleButton
    public void nextClimbStage(boolean enabled) {
        climbstage = climbstage + 1;
        switch(climbstage) {
            case 1:
                setpoint = ClimbConstants.kFullUpEncoderCount;
                setPosition(ClimbConstants.kFullUpEncoderCount);
                break;
            case 2:
                setpoint = ClimbConstants.kOnBarEncoderCount;
                setPosition(ClimbConstants.kOnBarEncoderCount);
                break;
            case 3:
                setpoint = ClimbConstants.kHangingEncoderCount;
                setPosition(ClimbConstants.kHangingEncoderCount);
                break;
            default:
        }
    }

    // Determines if the talon is at the desired position
    @Log
    public boolean atposition() {
        return inRange(m_LeftClimbMotor.getSelectedSensorPosition(), setpoint)
        && inRange(m_RightClimbMotor.getSelectedSensorPosition(), setpoint)
        && m_RightClimbMotor.getSelectedSensorPosition() > 100;
    }

    public Boolean inRange(double position, double setpoint){
        if(position > setpoint + ClimbConstants.kErrorTolerance){
            return false;
        }else if(position < setpoint - ClimbConstants.kErrorTolerance){
            return false;
        }else{
            return true;
        }
    }
}