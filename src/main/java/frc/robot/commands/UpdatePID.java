package frc.robot.commands;
import frc.robot.subsystems.AutoGenerator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDriveSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpdatePID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDriveSystem swervedrive;
 private final Shooter shooter;
 private final AutoGenerator autoGenerator;

  public UpdatePID(SwerveDriveSystem m_swervedrive, Shooter m_shooter,AutoGenerator m_autoGenerator) {
    swervedrive = m_swervedrive;
    shooter = m_shooter;
    autoGenerator=m_autoGenerator;
    addRequirements(swervedrive,shooter,m_autoGenerator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swervedrive.updateConstants();
    shooter.updateConstants();    
    autoGenerator.updateConstants();
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
