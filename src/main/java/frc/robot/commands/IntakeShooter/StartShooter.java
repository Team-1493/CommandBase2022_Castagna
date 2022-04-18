package frc.robot.commands.IntakeShooter;
import org.opencv.imgproc.IntelligentScissorsMB;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;



public class StartShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Shooter shooter;
  private int speed ;

  public StartShooter(Shooter m_shooter, int m_speed) {
    shooter=m_shooter;
    speed=m_speed;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
  shooter.shootAtSpeed(speed);
  }

  @Override
  public void execute() {
      shooter.shootAtSpeed(speed);
    }


  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
   return true;
  }
  
}   