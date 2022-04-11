package frc.robot.commands.LimelightFollowing;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.SwerveDriveSystem;

/***  This command gets called only once, so the method listed in 
       execute() should not end until it's done or a user signals it to stop.
     
***/

public class LinelightLineUp extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTable limelight= inst.getTable("limelight");
  public NetworkTableEntry txEntry = limelight.getEntry("tx");
  public NetworkTableEntry tvEntry = limelight.getEntry("tv");
  public NetworkTableEntry tyEntry = limelight.getEntry("ty");
  private SwerveDriveSystem sds;
  private double kP=0.1, kS=0.3;
  private double error=1;
  private JoystickButton btn;
  private Timer timer = new Timer();
  private boolean onTarget=false;
  private final Supplier<double[]> m_stickState;
  private double prev_angle = 0,delta_angle=0;

  private double horizantal_kP = 0.5;
  double coarseAngle=0;

  public LinelightLineUp(SwerveDriveSystem m_sds,JoystickButton m_btn,
   Supplier<double[]> stickState) {
      sds=m_sds;
      btn=m_btn;
      m_stickState = stickState;
      addRequirements(sds);
  }

  @Override
  public void initialize() {
    horizantal_kP = SmartDashboard.getNumber("Horizantal_kP", 0);
    timer.start();
    
    timer.reset();
    onTarget=false;
  }


  @Override
  public void execute() {
    sds.getChassisSpeed();
    double orientation = sds.getPose().getRotation().getDegrees();
    double seesTarget=tvEntry.getDouble(0);
    double angle=-txEntry.getDouble(0);
    double Vx = SmartDashboard.getNumber("Chassis xvel", 0); //forward (field oriented)
    double Vy = SmartDashboard.getNumber("Chassis yvel", 0);// sideways
    double direction = SmartDashboard.getNumber("Chassis angle", 0);//vel direction

    double Vel = Math.sqrt(Vx*Vx + Vy*Vy);
    double delta_angle = orientation + angle + direction;
    Vx = Vel * Math.cos(delta_angle);//forward
    Vy = Vel * Math.sin(delta_angle);//sideways



    //delta_angle=angle-prev_angle;
    //SmartDashboard.putNumber("prev_angle", prev_angle);
    double verticle_angle = tyEntry.getDouble(0);
    error = Math.abs(angle+horizantal_kP*Vy);
    SmartDashboard.putNumber("Limelight Vertical Angle", verticle_angle);
    SmartDashboard.putNumber("limelight angle", angle);
    //double dist = verticle_angle*angle_kp;
    double output;


    if(seesTarget==1 && error>0.5)
      {
        output = angle*kP; //+kS*Math.signum(angle);
        if (output>90) output=90;
        if (output<-90) output=-90;
        double driverstick[] = m_stickState.get();
        driverstick[2] = output;
        sds.setMotors(driverstick);
        onTarget=false;
        prev_angle = angle;
      }
  
    
    else{
      sds.setMotors(m_stickState.get());
    }
  }


      


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sds.setMotors(new double[] {0, 0,sds.heading, 3});

  }

  @Override
  public boolean isFinished() {
 return !btn.get(); //|| (onTarget && timer.hasElapsed(0.25)) );
  }
  
}