package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;



public class ClimbPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
  
  private Climber climber ;
  private int position;




  public ClimbPosition(Climber m_climber,int m_setting) {
    climber=m_climber;
    if(m_setting==1)position=0;
    else if (m_setting==2)position=-507569;
    else position=-1042522;
    
    
    addRequirements(climber);
  }
  
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    climber.climbPosition(position);
  }


  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return (true);
  }
  
}   