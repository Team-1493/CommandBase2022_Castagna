package frc.robot.commands.Gyro;

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