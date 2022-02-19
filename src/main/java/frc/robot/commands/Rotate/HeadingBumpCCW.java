package frc.robot.commands.Rotate;
import frc.robot.subsystems.SwerveDriveSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HeadingBumpCCW extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDriveSystem m_swervedrive;
 

  public HeadingBumpCCW(SwerveDriveSystem swervedrive) {
    m_swervedrive = swervedrive;
    addRequirements(swervedrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // supply vx,vy,heading setpoint
    m_swervedrive.headingBumpCCW();
  }

  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // m_swervedrive.setMotors(0.,0.,0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
  
}
