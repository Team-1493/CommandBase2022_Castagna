package frc.robot.commands.IntakeShooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeConveyor;



public class IntakeFirstBallAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeConveyor intakeConveyor;
  boolean ballTop=false,ballBottom=false;
  Timer timer = new Timer();
  

  public IntakeFirstBallAuto(IntakeConveyor m_intakeConveyor) {
    intakeConveyor=m_intakeConveyor;
    addRequirements(intakeConveyor);

  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
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
    return (ballTop && ballBottom);
  }
  
}   