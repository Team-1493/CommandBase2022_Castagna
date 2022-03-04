package frc.robot.commands.AutoCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeShooter.IntakeFirstBallAuto;
import frc.robot.commands.IntakeShooter.ShootBallAuto;
import frc.robot.subsystems.IntakeConveyor;
import frc.robot.subsystems.Shooter;



public class AutoActions4Ball extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private IntakeConveyor intakeConveyor;
  private Shooter shooter;
  boolean ballTop=false,ballBottom=false;
  boolean[] flags={false,false,false,false,false};
  Command intakeStart;
  Command shoot1,shoot2;
  Timer timer=new Timer();
  double endTime=9.47;

  public AutoActions4Ball(IntakeConveyor m_intakeConveyor, Shooter m_shooter) {
    shooter=m_shooter;
    intakeConveyor=m_intakeConveyor;
    intakeStart=new IntakeFirstBallAuto(intakeConveyor);
    shoot1=new ShootBallAuto(intakeConveyor,shooter,3,1600);
    shoot2=new ShootBallAuto(intakeConveyor,shooter,3,1750);
    addRequirements(intakeConveyor);

  }

  @Override
  public void initialize() {
      timer.start();
      timer.reset();
      intakeStart.schedule();
  }

  @Override
  public void execute() {
      if(timer.hasElapsed(1)&&!flags[0]){
          flags[0]=true;
          if(!intakeStart.isFinished()) intakeStart.cancel();
          shoot1.schedule();
      }
      if(timer.hasElapsed(3)&&!flags[1]){
        flags[1]=true;
        if(!shoot1.isFinished()) shoot1.cancel();
        intakeStart.schedule();
    }


  }

  @Override
  public void end(boolean interrupted) {
    if(!intakeStart.isFinished()) intakeStart.cancel();
    shoot1.schedule();
  }

  @Override
  public boolean isFinished() {
    return (timer.hasElapsed(endTime));
  }
  
}   