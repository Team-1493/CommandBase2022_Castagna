package frc.robot.commands.LimelightFollowing;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeShooter.ShootBall;
import frc.robot.subsystems.IntakeConveyor;
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
  private IntakeConveyor intake;
  private Command shoot;
  private Command lineUp;
  private double Angle_kP = 0.1;
  double coarseAngle=0;

  public LimelightMove(SwerveDriveSystem m_sds,JoystickButton m_btn,
   Supplier<double[]> stickState, Shooter m_shooter,IntakeConveyor m_intake) {
      sds=m_sds;
      btn=m_btn;
      intake=m_intake;
      m_stickState = stickState;
      shooter = m_shooter;
      addRequirements(sds,intake,shooter);

  }

  @Override
  public void initialize() {
    //timer.start();
    //timer.reset();
    //onTarget=false;
    shoot=new ShootBall(intake,shooter,btn,4);
    lineUp=new LinelightLineUp(sds, btn, m_stickState);
    lineUp.schedule();
  }


  @Override
  public void execute() {

    double seesTarget=tvEntry.getDouble(0);
    double angle= -txEntry.getDouble(0);
    error = Math.abs(angle);
    SmartDashboard.putNumber("limelight angle", angle);
    
    

    if(seesTarget ==1 && error<0.5){
      shoot.schedule();
      }  
    
      

    /*double vel = SmartDashboard.getNumber("Drive Vel", 0);
    double facing = SmartDashboard.getNumber("Orientation", 0);
    double xvel = Math.cos(facing)*vel;
    double angle_offset = Angle_kP*xvel;


    if(seesTarget==1 && error>0.5)
      {
        double output = angle*kP; //+kS*Math.signum(angle);
        if (output>90) output=90;
        if (output<-90) output=-90;
        double driverstick[] = m_stickState.get();
        driverstick[2] = output;
        sds.setMotors(driverstick);
       
      }
  
    else if(seesTarget ==1 && error<0.5){
      shoot.schedule();
      }  

    else{
      sds.setMotors(m_stickState.get());
    }*/
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