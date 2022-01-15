package frc.robot.commands;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.Utilities.DriverStationInterface;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdatePID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDriveSystem m_swervedrive;
 

  public UpdatePID(SwerveDriveSystem swervedrive) {
    m_swervedrive = swervedrive;
    addRequirements(swervedrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    DriverStationInterface.getNewConstants();
    m_swervedrive.updatePID();
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
