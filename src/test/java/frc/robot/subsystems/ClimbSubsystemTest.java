package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

import org.junit.*;

import static org.mockito.Mockito.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ClimbSubsystemTest {
    /* static {
        HLUsageReporting.SetImplementation(new HLUsageReporting.Null()); // CCB: Magic bits to turn off wpi usage reporting...nonsense dependency
    } */
    @Test
    public void itShouldExtendClimber()
    {
        // Assemble
        WPI_TalonSRX leftmock = mock(WPI_TalonSRX.class);
        WPI_TalonSRX rightmock = mock(WPI_TalonSRX.class);
        ClimbSubsystem climb = new ClimbSubsystem(leftmock, rightmock);

        // Act
        climb.nextClimbStage(true);

        // Assert
        verify(leftmock).set(ControlMode.Position, ClimbConstants.kFullUpEncoderCount);
    }
}