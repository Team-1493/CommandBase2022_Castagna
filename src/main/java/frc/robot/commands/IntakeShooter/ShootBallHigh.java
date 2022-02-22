package frc.robot.commands.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeConveyor;
import frc.robot.subsystems.Shooter;



public class ShootBallHigh extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeConveyor intakeConveyor;
  private Shooter shooter;
  private boolean ballTop=false,ballBottom=false,atSpeed=false;
  JoystickButton btn;
  private Timer timer = new Timer();
  private double startTime=0;
  private boolean noUpper=false;

  private boolean runUpper=false,runLower=false;
  

  public ShootBallHigh(IntakeConveyor m_intakeConveyor,Shooter m_shooter,JoystickButton m_btn) {
    btn=m_btn;
    intakeConveyor=m_intakeConveyor;
    shooter=m_shooter;
    addRequirements(intakeConveyor,shooter);
  }

  @Override
  public void initialize() {
      shooter.shootHigh();;
      runUpper=false;
      if (!intakeConveyor.ballAtTop()) noUpper=true;

      startTime=0;
      timer.start();
      timer.reset();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("timer", timer.get());
    SmartDashboard.putNumber("start time", startTime);

    atSpeed=shooter.atSpeed;

    if(atSpeed && !runUpper && (timer.get()-startTime)>2) {
      runUpper=true;
      intakeConveyor.startUpperConveyor();
      startTime=timer.get();
    }

    if(atSpeed && runUpper && (timer.get()-startTime>2) ) {
      intakeConveyor.startLowerConveyor();
    }
    
    if (!atSpeed)startTime=timer.get();  
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