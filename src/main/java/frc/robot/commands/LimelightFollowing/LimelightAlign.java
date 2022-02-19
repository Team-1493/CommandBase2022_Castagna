package frc.robot.commands.LimelightFollowing;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSystem;

/***  This command gets called only once, so the method listed in 
       execute() should not end until it's done or a user signals it to stop.
     
***/

public class LimelightAlign extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTable limelight= inst.getTable("limelight");
  public NetworkTableEntry txEntry = limelight.getEntry("tx");
  public NetworkTableEntry tvEntry = limelight.getEntry("tv");
  private SwerveDriveSystem sds;
  int loopsOnTarget=0;
  double allowableError=0.02;
  double error;
  double kP_lime=1;
  double minOmega=0.2;

  public LimelightAlign(SwerveDriveSystem m_sds) {
      sds=m_sds;
      SmartDashboard.putNumber("kP_lime", 1);
      SmartDashboard.putNumber("minOmega_lime", 0.2);

      addRequirements(sds);

  }

  @Override
  public void initialize() {
    error=0;
    loopsOnTarget=0;
    kP_lime=SmartDashboard.getNumber("kP_lime", 1);
    minOmega= SmartDashboard.getNumber("minOmega_lime", 0.2);
  }

  @Override
  public void execute() {

//    double angle=-txEntry.getDouble(0)/5 - 5;
   
    double angle=-txEntry.getDouble(0);
    double angleRadians=Math.toRadians(angle);
    double heading=sds.heading;
    SmartDashboard.putNumber("limelight angle", angle);
    if(tvEntry.getDouble(0)==1)
    sds.setMotors(new double[] {0, 0,heading+angleRadians, 3});    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  // add code to test when the command is done.
  // For example - is the robot close enough to the target,
  // or did autonomous end, or did the driver signal it to end
  @Override
  public boolean isFinished() {
    return (true);
  }
  
}