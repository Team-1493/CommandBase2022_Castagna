package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraInterface;

/***  This command gets called only once, so the method listed in 
       execute() should not end until it's done or a user signals it to stop.
     
***/

public class FollowCamera extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CameraInterface camInt;
  double[] prev_values = new double[3];
  

  public FollowCamera(CameraInterface m_camInt) {

      camInt=m_camInt;
      addRequirements(camInt);
      prev_values[0] = 0;
      prev_values[1] = 0;
      prev_values[2] = 0;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

      prev_values = camInt.driveToTartget(prev_values);     

      
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