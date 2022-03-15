package frc.robot.commands.Rotate;
import frc.robot.subsystems.SwerveDriveSystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class AlignWithField extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDriveSystem m_swervedrive;
  double initialHeading=0;
  double finalHeading=0;
  JoystickButton btn;
  Timer timer = new Timer();
 

  public AlignWithField(SwerveDriveSystem swervedrive) {
    m_swervedrive = swervedrive;
    addRequirements(swervedrive);
  }

  @Override
  public void initialize() {
    
    timer.start();
    timer.reset();
  }

  @Override
  public void execute() {
    // supply vx,vy,heading setpoint
//    m_swervedrive.headingBumpCW();
    m_swervedrive.setMotors(new double[] {0,0,0.,3});
  }

  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_swervedrive.allStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.hasElapsed(1) || Math.abs(m_swervedrive.heading)<=0.035);
  }
  
}
