package frc.robot.commands.IntakeShooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeConveyor;
import frc.robot.subsystems.Shooter;



public class ShootBallHighManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeConveyor intakeConveyor;
  private Shooter shooter;
  boolean ballTop=false,ballBottom=false,atSpeed=false;
  JoystickButton btn;

  public ShootBallHighManual(IntakeConveyor m_intakeConveyor,Shooter m_shooter,JoystickButton m_btn) {
    btn=m_btn;
    intakeConveyor=m_intakeConveyor;
    shooter=m_shooter;
    addRequirements(intakeConveyor,shooter);
  }

  @Override
  public void initialize() {
      shooter.shootManualHigh();
  }

  @Override
  public void execute() {
    atSpeed=shooter.atSpeed;
    ballTop=intakeConveyor.ballAtTop();
    ballBottom=intakeConveyor.ballAtBottom();

    if(ballTop && atSpeed) 
        intakeConveyor.startUpperConveyor();
    else intakeConveyor.stopUpperConveyor();

    if (ballBottom && !ballTop)
        intakeConveyor.startIntakeLowerConveyor();
    else
        intakeConveyor.stopLowerConveyor();
  }


  @Override
  public void end(boolean interrupted) {
        shooter.stopShooter();
        intakeConveyor.stopLowerConveyor();
        intakeConveyor.stopUpperConveyor();   
  }

  @Override
  public boolean isFinished() {
    return (!btn.get());
  }
  
}   