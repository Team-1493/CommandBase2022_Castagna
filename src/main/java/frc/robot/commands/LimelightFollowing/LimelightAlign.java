package frc.robot.commands.LimelightFollowing;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
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
  private double kP=0.115, kS=0.3;
  private double error=1;
  private JoystickButton btn;
  private Timer timer = new Timer();
  private boolean onTarget=false;
  double coarseAngle=0;

  public LimelightAlign(SwerveDriveSystem m_sds,JoystickButton m_btn) {
      sds=m_sds;
      btn=m_btn;
      addRequirements(sds);

  }

  @Override
  public void initialize() {
    timer.start();
    timer.reset();
    onTarget=false;

  }


  @Override
  public void execute() {

    double seesTarget=tvEntry.getDouble(0);
    double angle=-txEntry.getDouble(0);
    error = Math.abs(angle);
    SmartDashboard.putNumber("limelight angle", angle);
   /*
    if(seesTarget==0){
      double x = sds.getPose().getX() - 8.27;
      double y = sds.getPose().getY()-4.28;
      if(x>0 && y<0) coarseAngle = Math.PI - Math.atan(x/y);
      else if(x<0 && y<0) coarseAngle = - Math.atan(x/y);
      else if(x>0 && y>0) coarseAngle=-180-Math.atan(x/y);
      else coarseAngle = - Math.atan(x/y);
      sds.setMotors(new double[] {0, 0,coarseAngle, 3});
    }
*/

    if(seesTarget==1 && error>0.5)
      {
        double output = angle*kP+kS*Math.signum(angle);
        if (output>90) output=90;
        if (output<-90) output=-90;
        sds.setMotors(new double[] {0, 0,output, 1});
        onTarget=false;
      }
  
    else if(seesTarget==1 && error<= 0.5 && !onTarget)  
      {
        timer.reset();
        onTarget=true;
      }   
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sds.setMotors(new double[] {0, 0,sds.heading, 3});
  }

  @Override
  public boolean isFinished() {
 return (!btn.get() || (onTarget && timer.hasElapsed(0.25)) );
  }
  
}