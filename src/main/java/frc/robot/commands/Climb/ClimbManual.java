package frc.robot.commands.Climb;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climber;



public class ClimbManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Climber climber ;
  private int direction;
  private JoystickButton btn;


  public ClimbManual(Climber m_climber,JoystickButton _btn,int m_direcion) {
    climber=m_climber;
    btn=_btn;
    direction=m_direcion;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (direction==1) climber.climbUp();
    else climber.climbDown();
  }


  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return (!btn.get());
  }
  
}   