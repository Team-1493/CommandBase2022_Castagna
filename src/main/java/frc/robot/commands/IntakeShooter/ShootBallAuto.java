package frc.robot.commands.IntakeShooter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeConveyor;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;



public class ShootBallAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public IntakeConveyor intakeConveyor;
  public Shooter shooter;
  public boolean atSpeed=false;
  public boolean runUpper=false;
  public boolean runLower=false;
  public int shooterLevel;
  public Timer timer = new Timer();
  int rpm=1000;

  public ShootBallAuto(IntakeConveyor m_intakeConveyor,Shooter m_shooter, int m_shooterLevel) {
    intakeConveyor=m_intakeConveyor;
    shooter=m_shooter;
    shooterLevel=m_shooterLevel;    
    addRequirements(intakeConveyor,shooter);
  }

  public ShootBallAuto(IntakeConveyor m_intakeConveyor,Shooter m_shooter, int m_shooterLevel,int m_rpm) {
    intakeConveyor=m_intakeConveyor;
    shooter=m_shooter;
    shooterLevel=m_shooterLevel;    
    rpm=m_rpm;
    addRequirements(intakeConveyor,shooter);
  }



  @Override
  public void initialize() {
      if(shooterLevel==1) shooter.shootHigh();
      else if(shooterLevel==2) shooter.shootLow();
      else shooter.shootAtSpeed(rpm); 
      runUpper=false;
      runLower=false;
  }

  @Override
  public void execute() {
    atSpeed=shooter.atSpeed;

    if(atSpeed && !runUpper) {
      runUpper=true;
      intakeConveyor.startUpperConveyor();
    }

    if(atSpeed && runUpper && !runLower) {
       runLower=true;
      intakeConveyor.startLowerConveyor();
      timer.reset();
      timer.start();
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
    return (timer.hasElapsed(0.5));
  }
  
}   