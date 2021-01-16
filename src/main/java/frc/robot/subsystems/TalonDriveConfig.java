package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import frc.robot.Constants.DriveConstants;

public class TalonDriveConfig {
   
   public TalonDriveConfig (WPI_TalonSRX m_talonsrxleft, WPI_TalonSRX m_talonsrxright, PigeonIMU m_pigeon) {
      /* Disable all motor controllers */
      m_talonsrxright.set(0);
      m_talonsrxleft.set(0);
      
      // Set Ramping
      m_talonsrxleft.configClosedloopRamp(DriveConstants.kClosedRamp);
      m_talonsrxleft.configOpenloopRamp(DriveConstants.kOpenRamp);
      m_talonsrxright.configClosedloopRamp(DriveConstants.kClosedRamp);
      m_talonsrxright.configOpenloopRamp(DriveConstants.kOpenRamp);

      /* Set Neutral Mode */
      m_talonsrxleft.setNeutralMode(NeutralMode.Brake);
      m_talonsrxright.setNeutralMode(NeutralMode.Brake);
      
      /** Feedback Sensor Configuration */
      
      /* Configure the Pigeon IMU as a Remote Sensor for the right Talon */
      m_talonsrxright.configRemoteFeedbackFilter(m_pigeon.getDeviceID(),			      // Device ID of Source
                                    RemoteSensorSource.GadgeteerPigeon_Yaw,	         // Remote Feedback Source
                                    DriveConstants.REMOTE_1,				               // Remote number [0, 1]
                                    DriveConstants.kTimeoutMs);			               // Configuration Timeout
      
      /* Configure the Remote Sensor to be the Selected Sensor of the right Talon */
      m_talonsrxright.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1, 	// Set remote sensor to be used directly
                                       DriveConstants.PID_TURN, 			               // PID Slot for Source [0, 1]
                                       DriveConstants.kTimeoutMs);		               // Configuration Timeout
      
      /* Scale the Selected Sensor using a coefficient (Values explained in Constants.java */
      m_talonsrxright.configSelectedFeedbackCoefficient(	DriveConstants.kTurnTravelUnitsPerRotation / DriveConstants.kPigeonUnitsPerRotation,	// Coefficient
                           DriveConstants.PID_TURN, 							               // PID Slot of Source
                           DriveConstants.kTimeoutMs);						               // Configuration Timeout
      
      /* Set status frame periods */
      m_talonsrxright.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, DriveConstants.kTimeoutMs);
      m_talonsrxright.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, DriveConstants.kTimeoutMs);
      m_talonsrxright.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, DriveConstants.kTimeoutMs);
      m_talonsrxleft.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, DriveConstants.kTimeoutMs);
      m_talonsrxleft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, DriveConstants.kTimeoutMs);
      m_talonsrxleft.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, DriveConstants.kTimeoutMs);
      m_pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, DriveConstants.kTimeoutMs);
      
      /* max out the peak output (for all modes).  However you can
      * limit the output of a given PID object with configClosedLoopPeakOutput().
      */
      m_talonsrxleft.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
      m_talonsrxleft.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);
      m_talonsrxright.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
      m_talonsrxright.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);

      /* FPID Gains for distance servo */
      m_talonsrxright.config_kP(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kP, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kI(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kI, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kD(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kD, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kF(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kF, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_IntegralZone(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kIzone, DriveConstants.kTimeoutMs);
      m_talonsrxright.configClosedLoopPeakOutput(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kPeakOutput, DriveConstants.kTimeoutMs);

      /* FPID Gains for turn servo */
      m_talonsrxright.config_kP(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kP, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kI(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kI, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kD(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kD, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kF(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kF, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_IntegralZone(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kIzone, DriveConstants.kTimeoutMs);
      m_talonsrxright.configClosedLoopPeakOutput(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kPeakOutput, DriveConstants.kTimeoutMs);
      m_talonsrxright.configAllowableClosedloopError(DriveConstants.kSlot_Turning, 0, DriveConstants.kTimeoutMs);

      /* FPID Gains for velocity servo */
      m_talonsrxright.config_kP(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kP, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kI(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kI, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kD(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kD, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kF(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kF, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_IntegralZone(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kIzone, DriveConstants.kTimeoutMs);
      m_talonsrxright.configClosedLoopPeakOutput(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kPeakOutput, DriveConstants.kTimeoutMs);
      m_talonsrxright.configAllowableClosedloopError(DriveConstants.kSlot_Velocit, 0, DriveConstants.kTimeoutMs);	

      /* FPID Gains for motion profiling servo */
      m_talonsrxright.config_kP(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kP, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kI(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kI, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kD(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kD, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_kF(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kF, DriveConstants.kTimeoutMs);
      m_talonsrxright.config_IntegralZone(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kIzone, DriveConstants.kTimeoutMs);
      m_talonsrxright.configClosedLoopPeakOutput(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kPeakOutput, DriveConstants.kTimeoutMs);
      m_talonsrxright.configAllowableClosedloopError(DriveConstants.kSlot_MotProf, 0, DriveConstants.kTimeoutMs);	

      /* FPID Gains for distance servo */
      m_talonsrxleft.config_kP(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kP, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kI(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kI, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kD(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kD, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kF(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kF, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_IntegralZone(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kIzone, DriveConstants.kTimeoutMs);
      m_talonsrxleft.configClosedLoopPeakOutput(DriveConstants.kSlot_Distanc, DriveConstants.kGains_Distanc.kPeakOutput, DriveConstants.kTimeoutMs);

      /* FPID Gains for turn servo */
      m_talonsrxleft.config_kP(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kP, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kI(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kI, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kD(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kD, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kF(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kF, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_IntegralZone(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kIzone, DriveConstants.kTimeoutMs);
      m_talonsrxleft.configClosedLoopPeakOutput(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kPeakOutput, DriveConstants.kTimeoutMs);
      m_talonsrxleft.configAllowableClosedloopError(DriveConstants.kSlot_Turning, 0, DriveConstants.kTimeoutMs);

      /* FPID Gains for velocity servo */
      m_talonsrxleft.config_kP(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kP, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kI(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kI, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kD(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kD, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kF(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kF, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_IntegralZone(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kIzone, DriveConstants.kTimeoutMs);
      m_talonsrxleft.configClosedLoopPeakOutput(DriveConstants.kSlot_Velocit, DriveConstants.kGains_Velocit.kPeakOutput, DriveConstants.kTimeoutMs);
      m_talonsrxleft.configAllowableClosedloopError(DriveConstants.kSlot_Velocit, 0, DriveConstants.kTimeoutMs);	

      /* FPID Gains for motion profiling servo */
      m_talonsrxleft.config_kP(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kP, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kI(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kI, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kD(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kD, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_kF(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kF, DriveConstants.kTimeoutMs);
      m_talonsrxleft.config_IntegralZone(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kIzone, DriveConstants.kTimeoutMs);
      m_talonsrxleft.configClosedLoopPeakOutput(DriveConstants.kSlot_MotProf, DriveConstants.kGains_MotProf.kPeakOutput, DriveConstants.kTimeoutMs);
      m_talonsrxleft.configAllowableClosedloopError(DriveConstants.kSlot_MotProf, 0, DriveConstants.kTimeoutMs);	
      
      m_talonsrxleft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
      m_talonsrxleft.configVelocityMeasurementWindow(1);

      m_talonsrxright.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
      m_talonsrxright.configVelocityMeasurementWindow(1);

      /* 1ms per loop.  PID loop can be slowed down if need be.
      * For example,
      * - if sensor updates are too slow
      * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
      * - sensor movement is very slow causing the derivative error to be near zero.
      */
      final int closedLoopTimeMs = 1;
      m_talonsrxright.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.kTimeoutMs);
      m_talonsrxright.configClosedLoopPeriod(1, closedLoopTimeMs, DriveConstants.kTimeoutMs);

      m_talonsrxleft.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.kTimeoutMs);
      m_talonsrxleft.configClosedLoopPeriod(1, closedLoopTimeMs, DriveConstants.kTimeoutMs);

      /*
      * configAuxPIDPolarity(boolean invert, int timeoutMs) false means talon's local
      * output is PID0 + PID1, and other side Talon is PID0 - PID1 true means talon's
      * local output is PID0 - PID1, and other side Talon is PID0 + PID1
      */
      m_talonsrxright.configAuxPIDPolarity(false, DriveConstants.kTimeoutMs);
      m_talonsrxleft.configAuxPIDPolarity(false, DriveConstants.kTimeoutMs);
   }

}
