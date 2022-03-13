package frc.robot.commands.IntakeShooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeConveyor;



public class LowerIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeConveyor intake;
private Timer timer = new Timer();
  

  public LowerIntake(IntakeConveyor m_intake) {
    intake = m_intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  @Override
  public void execute() {
      intake.reverseIntake();
    }


  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();;
  }

  @Override
  public boolean isFinished() {
    return (timer.hasElapsed(0.3));
  }
  
}   