package frc.robot.Utilities;
import frc.robot.subsystems.SwerveModuleMDK;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Constants_Swerve.*;

public class ModuleGenerator{
  
public ModuleGenerator(){
}

public SwerveModuleMDK[] generateModuleMDK(){
    
  SwerveModuleMDK[] modules = new SwerveModuleMDK[4];

 // front left swerve module
 modules[0] = new SwerveModuleMDK(
   frDriveID,
   frTurnID,
   frCancoderID,
   turnMotorZeroPos[0],
   invertDR,
   invertTR,
   invertEncDR,
   invertEncTR
   );

// front right swerve module     
   modules[1] = new SwerveModuleMDK(
   flDriveID,
   flTurnID,
   flCancoderID,
   turnMotorZeroPos[1],
   invertDL,
   invertTL,
   invertEncDL,
   invertEncTL
      );

// back left swerve module         
 modules[2] = new SwerveModuleMDK(
   brDriveID,
   brTurnID,
   brCancoderID,
   turnMotorZeroPos[2],
   invertDR,
   invertTR,
   invertEncDR,
   invertEncTR
   );

// back right swerve module      
 modules[3] = new SwerveModuleMDK(
   blDriveID,
   blTurnID,
   blCancoderID,
   turnMotorZeroPos[3],
   invertDL,
   invertTL,
   invertEncDL,
   invertEncTL
   );

 return modules;
}



public void updatePID(SwerveModuleMDK[] modules) {
  // update modules (the rotate PID is updated in SwerveDriveSystem)
    int i=0;
    while (i<4){
      modules[i].setTurnPID();
      modules[i].setDrivePID();
      i++;
    }
  }

}