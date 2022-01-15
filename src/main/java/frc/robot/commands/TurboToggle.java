package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Stick;

public class TurboToggle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Stick m_stick;
  
  public TurboToggle(Stick stick) {
    m_stick = stick;
    addRequirements(m_stick);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_stick.turboToggle();
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
