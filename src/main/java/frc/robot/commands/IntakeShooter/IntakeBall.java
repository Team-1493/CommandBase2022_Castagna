package frc.robot.commands.IntakeShooter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeConveyor;



public class IntakeBall extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeConveyor intakeConveyor;
  boolean ballTop=false,ballBottom=false;
  JoystickButton btn;

  public IntakeBall(IntakeConveyor m_intakeConveyor,JoystickButton m_btn) {
      btn=m_btn;
    intakeConveyor=m_intakeConveyor;
    addRequirements(intakeConveyor);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ballTop=intakeConveyor.ballAtTop();
    ballBottom=intakeConveyor.ballAtBottom();

    if(!ballTop) 
        intakeConveyor.startUpperConveyor();
    else intakeConveyor.stopUpperConveyor();

    if (!ballBottom || !ballTop){
        intakeConveyor.startIntake();
        intakeConveyor.startIntakeLowerConveyor();
    }
    else{
        intakeConveyor.stopIntake();
        intakeConveyor.stopLowerConveyor();
      }

  }

  @Override
  public void end(boolean interrupted) {
        intakeConveyor.stopIntake();
        intakeConveyor.stopLowerConveyor();
        intakeConveyor.stopUpperConveyor();
   
  }

  @Override
  public boolean isFinished() {
    return (!btn.get());
  }
  
}   