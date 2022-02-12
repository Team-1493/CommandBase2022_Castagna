package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallFollowInterface;
import frc.robot.subsystems.LimelightInterface;

/***  This command gets called only once, so the method listed in 
       execute() should not end until it's done or a user signals it to stop.
     
***/

public class FollowLimelight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimelightInterface limelightInterface;
  double[] prev_values = new double[3];
  

  public FollowLimelight(LimelightInterface m_limelightInterface) {

    limelightInterface=m_limelightInterface;
      addRequirements(limelightInterface);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    limelightInterface.turnToTarget();     

      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
  }

  // Returns true when the command should end.
  // add code to test when the command is done.
  // For example - is the robot close enough to the target,
  // or did autonomous end, or did the driver signal it to end
  @Override
  public boolean isFinished() {
    return true;
  }
  
}