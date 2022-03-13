package frc.robot.commands.Rotate;
import frc.robot.subsystems.SwerveDriveSystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HeadingBumpCW extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDriveSystem m_swervedrive;
  double initialHeading=0;
  double finalHeading=0;
  Timer timer = new Timer();
 

  public HeadingBumpCW(SwerveDriveSystem swervedrive) {
    m_swervedrive = swervedrive;
    addRequirements(swervedrive);
  }

  @Override
  public void initialize() {
    initialHeading=m_swervedrive.heading;
    finalHeading=initialHeading-0.026;  // 1.5 degrees
    
    timer.start();
    timer.reset();
  }

  @Override
  public void execute() {
    // supply vx,vy,heading setpoint
//    m_swervedrive.headingBumpCW();
    m_swervedrive.setMotors(new double[] {0,0,-0.5,1});
  }

  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // m_swervedrive.setMotors(0.,0.,0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.hasElapsed(.25) || m_swervedrive.heading<=finalHeading);
  }
  
}
