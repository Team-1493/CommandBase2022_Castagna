package frc.robot.commands.Gyro;

import java.util.function.Supplier;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSystem;

/***  This command gets called only once, so the method listed in 
       execute() should not end until it's done or a user signals it to stop.
     
***/

public class ReEnableGyro {
  private SwerveDriveSystem sds;

  public ReEnableGyro(SwerveDriveSystem m_sds) {
      sds=m_sds;

  }
  

  public void resetGryoAndRobotHeading() {
//
    double heading =sds.heading;
     sds.setMotors(new double[] {0, 0,heading, 3});    
  }
  
}