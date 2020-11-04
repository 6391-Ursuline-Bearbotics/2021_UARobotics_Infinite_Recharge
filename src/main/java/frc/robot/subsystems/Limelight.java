package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase implements Loggable{
  /**
   * Creates a new Limelight.
   */
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");
    private NetworkTableEntry tv = table.getEntry("ta");
    private NetworkTableEntry ledMode = table.getEntry("ledMode");
    private int loop;
    private boolean m_targeting = false;


  public Limelight() {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */

    
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.ta = table.getEntry("tv");
    this.ledMode = table.getEntry("ledMode");
    this.loop = 0; 
    setPipeline(LimelightConstants.DRIVE_PIPELINE);
    setLedOn(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //read values periodically
    double x = this.tx.getDouble(0.0);
    double y = this.ty.getDouble(0.0);
    double area = this.ta.getDouble(0.0);


    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

  }

  /**Returns distance to target in inches */
  public double distanceToTargetInInches(){
    double cameraAngle = LimelightConstants.CAMERA_ANGLE; 
    double angleToTarget = this.tx.getDouble(0.0);
    double camHeight = LimelightConstants.CAMERA_HEIGHT;
    double targetHeight = LimelightConstants.TARGET_HEIGHT;
    double distance =  ((targetHeight-camHeight) / Math.tan(cameraAngle+angleToTarget));
    
    return distance;
    
  }

  /** Returns if limelight can see defined retroreflective target */
  @Log
  public boolean hasTarget(){
   // this.table.getEntry("ledMode").setNumber(3);
    SmartDashboard.putNumber("tv; ", tv.getDouble(0));
    if(tv.getDouble(0) != 0)
      return true;
    

    return false;
  }

  @Log
  public double getTx() {
    return tx.getDouble(0);
  }

  public NetworkTableEntry getTy() {
    return ty;
  }

  public NetworkTableEntry getTa() {
    return ta;
  }

  public void setPipeline(int pipeline){
    this.table.getEntry("pipeline").setNumber(pipeline);
  }

  public void setStream(int stream){
    this.table.getEntry("stream").setNumber(stream);
  }

  public void setLedOn(boolean isOn) {
    if (isOn){
      ledMode.setNumber(LimelightConstants.LED_ON);
    } else {
      ledMode.setNumber(LimelightConstants.LED_OFF);
    }
  }
  
  /**Returns the angle to targed in degrees negative values to the left and positive to the right
   * used for turn to target
   */
  @Log
  public double getAngleOfError(){
    //+1 is a fudge factor cor camera mounting
    return getTx();
  }

  public void autoTurnToTarget(DriveSubsystem drivetrain, ShooterSubsystem shooter){
    while (turnToTargetVolts(drivetrain,shooter));
  }
  public boolean turnToTargetVolts(DriveSubsystem drivetrain, ShooterSubsystem shooter){
    //SmartDashboard.putString("turnToTarget ","Started");
    double turn = 0;
    double min = 3.5;
    boolean check = hasTarget();
    /* if (hasTarget() && !shooter.atSetpoint()){
      shooter.setSetpoint(ShooterConstants.kShooterFarTrenchRPM);
    } */
   // SmartDashboard.putString("Target ","" + check);
    //SmartDashboard.putString("Initial Tx","" + getAngleOfError());
    if(Math.abs(getAngleOfError()) >= LimelightConstants.TURN_TO_TARGET_TOLERANCE && hasTarget()){
      turn = getAngleOfError()*0.35;     
      if (Math.abs(turn) < min){
        turn = turn > 0 ? min:-min;
      }
      //SmartDashboard.putNumber("Turn", turn);
      drivetrain.tankDriveVolts(turn, -turn);
      //SmartDashboard.putString("Loop Tx:",this.loop++ + ":" + getAngleOfError());
    } else {
      //This prevents errors on the console from not updated drive train.
      drivetrain.tankDriveVolts(0, 0);
    }
    //SmartDashboard.putString("Ending TY","" + getAngleOfError());
    //SmartDashboard.putBoolean("Turning Complete",! (Math.abs(getAngleOfError()) >= LimelightConstants.TURN_TO_TARGET_TOLERANCE && hasTarget()));
    return !(Math.abs(getAngleOfError()) >= LimelightConstants.TURN_TO_TARGET_TOLERANCE) && hasTarget();
  }

  public void beforeTurnToTarget(){
    setLedOn(true);
    //switchPipeline(true);
    setStream(1);
    m_targeting = true;
  }

  public void afterTurnToTarget(){
    setLedOn(false);
    //switchPipeline(false);
    setStream(2);
    m_targeting = false;
  }
  public void switchPipeline(boolean targeting){
    if(targeting == true){
      setPipeline(LimelightConstants.TARGET_PIPELINE);
    } else {
      setPipeline(LimelightConstants.DRIVE_PIPELINE);
    }
  }
  
  @Log
  public boolean isTargeting(){
    return m_targeting;
  }

  @Log
  public boolean isOutOfRange(){
    return (getTy().getDouble(0) > LimelightConstants.RANGE_TOO_CLOSE || getTy().getDouble(0) < LimelightConstants.RANGE_TOO_FAR);
  }

  public boolean isPrimeRange(){
    return (getTy().getDouble(0) > LimelightConstants.RANGE_PRIME_END && getTy().getDouble(0) < LimelightConstants.RANGE_PRIME_START);
  }
}