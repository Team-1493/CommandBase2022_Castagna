package frc.robot.commands.IntakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeConveyor;
import frc.robot.subsystems.Shooter;



public class SpinWheel extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Shooter shooter;
  JoystickButton btn;

  

  public SpinWheel(Shooter m_shooter,JoystickButton m_btn) {
    btn=m_btn;
    shooter=m_shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
      shooter.shootManual();
    }


  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return (!btn.get());
  }
  
}   