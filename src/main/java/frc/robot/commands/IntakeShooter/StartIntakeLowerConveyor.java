package frc.robot.commands.IntakeShooter;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeConveyor;



public class StartIntakeLowerConveyor extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeConveyor m_intakeConveyor;


  public StartIntakeLowerConveyor(IntakeConveyor intakeConveyor) {
    m_intakeConveyor=intakeConveyor;
      addRequirements(intakeConveyor);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_intakeConveyor.startIntakeLowerConveyor();
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

    return (m_intakeConveyor.ballAtBottom());
  }
  
}   

