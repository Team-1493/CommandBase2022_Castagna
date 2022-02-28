package frc.robot.commands;
import frc.robot.subsystems.SwerveDriveSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetPose extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDriveSystem swervedrive;
  private final Pose2d pose;
 

  public ResetPose(SwerveDriveSystem m_swervedrive, Pose2d m_pose) {
    swervedrive = m_swervedrive;
    pose=m_pose;
    addRequirements(swervedrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swervedrive.resetOdometry(pose);
  }

  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
  
}
