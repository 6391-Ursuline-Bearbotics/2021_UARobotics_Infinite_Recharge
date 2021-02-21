package frc.robot.subsystems;

import static org.junit.Assert.*;
import org.junit.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ClimbSubsystemTest {
    ClimbSubsystem m_climber;
    TalonSRXSimCollection m_climberLeftSim;
    TalonSRXSimCollection m_climberRightSim;

    @Before // this method will run before each test
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        m_climber = new ClimbSubsystem(); // create our climber
        m_climberLeftSim = m_climber.getLeftSim();
        m_climberRightSim = m_climber.getRightSim();
    }

    @After // this method will run after each test
    public void shutdown() throws Exception {
        m_climber.close(); // destroy our intake object
    }

    @Test
    public void itShouldExtendClimber()
    {
        // Act
        m_climber.nextClimbStage(true);

        // Assert
        m_climberLeftSim.
    }
}