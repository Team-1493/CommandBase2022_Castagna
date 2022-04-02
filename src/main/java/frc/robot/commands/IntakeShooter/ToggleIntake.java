package frc.robot.commands.IntakeShooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeConveyor;



public class ToggleIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeConveyor intake;
  

  public ToggleIntake(IntakeConveyor m_intake) {
    intake = m_intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      intake.toggleIntake();
    }


  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
  
}   