package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;



public class AutoZero extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Climber climber;


  public AutoZero(Climber m_climber) {
    climber=m_climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    climber.climbDown();
  }


  @Override
  public void end(boolean interrupted) {
    climber.stop();
    climber.setPositionToZero();
  }

  @Override
  public boolean isFinished() {
    return (climber.getLeftLimitSwitch()==1 && climber.getRightLimitSwitch()==1);
  }
  
}   