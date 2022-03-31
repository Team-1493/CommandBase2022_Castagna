package frc.robot.commands.LimelightFollowing;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDriveSystem;

/***  This command gets called only once, so the method listed in 
       execute() should not end until it's done or a user signals it to stop.
     
***/

public class LimelightMove extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTable limelight= inst.getTable("limelight");
  public NetworkTableEntry txEntry = limelight.getEntry("tx");
  public NetworkTableEntry tvEntry = limelight.getEntry("tv");
  private SwerveDriveSystem sds;
  private double kP=0.055, kS=0.3;
  private double error=1;
  private JoystickButton btn;
  private Timer timer = new Timer();
  private boolean onTarget=false;
  private final Supplier<double[]> m_stickState;
  private Shooter shooter;
  double coarseAngle=0;

  public LimelightMove(SwerveDriveSystem m_sds,JoystickButton m_btn, Supplier<double[]> stickState, Shooter m_shooter) {
      sds=m_sds;
      btn=m_btn;
      m_stickState = stickState;
      shooter = m_shooter;
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
        double output = angle*kP; //+kS*Math.signum(angle);
        if (output>90) output=90;
        if (output<-90) output=-90;
        double driverstick[] = m_stickState.get();
        driverstick[2] = output;
        sds.setMotors(driverstick);
        onTarget=false;
      }
  
    else if(seesTarget==1 && error<= 0.5 && !onTarget)  
      {
        timer.reset();
        onTarget=true;
        sds.setMotors(m_stickState.get());
      }
    else{
      sds.setMotors(m_stickState.get());
    }

    //run the shooter motors to specified velocity according to the limelight's reading
    if(seesTarget==1){
      shooter.shootHigh();
    }
    else{
      shooter.stopShooter();
    }
  }


      


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sds.setMotors(new double[] {0, 0,sds.heading, 3});
    shooter.stopShooter();

  }

  @Override
  public boolean isFinished() {
 return !btn.get(); //|| (onTarget && timer.hasElapsed(0.25)) );
  }
  
}