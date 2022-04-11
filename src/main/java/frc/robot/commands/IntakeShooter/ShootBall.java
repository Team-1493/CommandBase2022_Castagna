package frc.robot.commands.IntakeShooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeConveyor;
import frc.robot.subsystems.Shooter;



public class ShootBall extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeConveyor intakeConveyor;
  private Shooter shooter;
  private boolean atSpeed=false;
  private JoystickButton btn;
  private boolean runUpper=false;
  private int shooterLevel;
  private Timer timer = new Timer();

  public ShootBall(IntakeConveyor m_intakeConveyor,Shooter m_shooter,JoystickButton m_btn, int m_shooterLevel) {
    btn=m_btn;
    intakeConveyor=m_intakeConveyor;
    shooter=m_shooter;
    shooterLevel=m_shooterLevel;
    timer.start();
    addRequirements(intakeConveyor,shooter);
  }

  @Override
  public void initialize() {
      if(shooterLevel==1) shooter.shootHigh();
      else if(shooterLevel==2) shooter.shootLow();
      else if(shooterLevel==3) shooter.shootManual();
      else shooter.shootMoving();      
      runUpper=false;
  }

  @Override
  public void execute() {

    atSpeed=shooter.atSpeed;

    if(atSpeed && !runUpper) {
      System.out.println("run upper");
      runUpper=true;
      intakeConveyor.startUpperConveyor();
      timer.reset();
    }

    else if(atSpeed && runUpper && timer.hasElapsed(0.1) ) {
      intakeConveyor.startLowerConveyor();
    }
    
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