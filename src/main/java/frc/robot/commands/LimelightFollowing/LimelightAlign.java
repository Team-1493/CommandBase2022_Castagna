package frc.robot.commands.LimelightFollowing;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private double kP=0.1, kS=0.1;
  private double error=1;
  private JoystickButton btn;
  

  public LimelightAlign(SwerveDriveSystem m_sds,JoystickButton m_btn) {
      sds=m_sds;
      btn=m_btn;
      addRequirements(sds);

  }

  @Override
  public void initialize() {
  }


  @Override
  public void execute() {

//    double angle=-txEntry.getDouble(0)/5 - 5;
   
    double angle=-txEntry.getDouble(0);
    error = Math.abs(angle);
    SmartDashboard.putNumber("limelight angle", angle);
   
    if(tvEntry.getDouble(0)==1 && error>0.5)
      sds.setMotors(new double[] {0, 0,angle*kP+kS, 1});
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sds.setMotors(new double[] {0, 0,0, 1});
  }

  @Override
  public boolean isFinished() {
 return (!btn.get());
  }
  
}