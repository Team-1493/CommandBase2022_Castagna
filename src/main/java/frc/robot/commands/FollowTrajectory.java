package frc.robot.commands;
import frc.robot.subsystems.SwerveDriveSystem;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTrajectory extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDriveSystem m_swervedrive;
  private IntSupplier m_povSelection;

  public FollowTrajectory(SwerveDriveSystem swervedrive, IntSupplier povSelection) {
    m_swervedrive = swervedrive;
    m_povSelection=povSelection;
    addRequirements(m_swervedrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // supply vx,vy,heading setpoint

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
