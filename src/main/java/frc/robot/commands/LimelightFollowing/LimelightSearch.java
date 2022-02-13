package frc.robot.commands.LimelightFollowing;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSystem;

/***  This command gets called only once, so the method listed in 
       execute() should not end until it's done or a user signals it Supplier<double[]> stickStateto stop.
     
***/

public class LimelightSearch extends CommandBase {
  private final SwerveDriveSystem sds;
  private final Supplier<double[]> stickState;
  boolean firstCall=false;
  double startHeading;
  int direction=1;
  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTable limelight= inst.getTable("limelight");
  public NetworkTableEntry tvEntry = limelight.getEntry("tv");

  public LimelightSearch(SwerveDriveSystem m_sds, Supplier<double[]> m_stickState) {
      sds=m_sds;
      stickState=m_stickState;
      addRequirements(sds);
  }

  @Override
  public void initialize() {
    firstCall=false;
    startHeading=sds.heading;
    direction=1;
  }

  @Override
  public void execute() {
    sds.setMotors(new double[] {0,0,0.3*direction,2});
    if(direction==1 && sds.heading>(startHeading+Math.PI/6))
      direction=-1;
    if(direction==-1 && sds.heading<(startHeading+Math.PI/6))
      end(true);  

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
    return (tvEntry.getDouble(0)==1 || stickState.get()[1]>.1 || stickState.get()[0]>.1 );
  }
  
}