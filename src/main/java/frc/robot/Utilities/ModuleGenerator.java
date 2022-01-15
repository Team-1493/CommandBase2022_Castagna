package frc.robot.Utilities;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveModuleMDK;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Constants_Swerve.*;

public class ModuleGenerator{
  
public ModuleGenerator(){
}

  public SwerveModule[] generateModule(){
    
     SwerveModule[] modules = new SwerveModule[4];
   
    // front left swerve module
    modules[0] = new SwerveModule(
      flDriveID,
      flTurnID,
      invertDL,
      invertTL,
      invertEncDL,
      invertEncTL
      );

 // front right swerve module     
      modules[1] = new SwerveModule(
      frDriveID,
      frTurnID,
      invertDR,
      invertTR,
      invertEncDR,
      invertEncTR
         );

// back left swerve module         
    modules[2] = new SwerveModule(
      blDriveID,
      blTurnID,
      invertDL,
      invertTL,
      invertEncDL,
      invertEncTL
      );

// back right swerve module      
    modules[3] = new SwerveModule(
      brDriveID,
      brTurnID,
      invertDR,
      invertTR,
      invertEncDR,
      invertEncTR
      );
System.out.println(blDriveID+" "+blTurnID+" "+brTurnID+" "+brTurnID);
    return modules;
}



public SwerveModuleMDK[] generateModuleMDK(){
    
  SwerveModuleMDK[] modules = new SwerveModuleMDK[4];

 // front left swerve module
 modules[0] = new SwerveModuleMDK(
   flDriveID,
   flTurnID,
   flCancoderID,
   turnMotorZeroPos[0],
   invertDL,
   invertTL,
   invertEncDL,
   invertEncTL
   );

// front right swerve module     
   modules[1] = new SwerveModuleMDK(
   frDriveID,
   frTurnID,
   frCancoderID,
   turnMotorZeroPos[1],
   invertDR,
   invertTR,
   invertEncDR,
   invertEncTR
      );

// back left swerve module         
 modules[2] = new SwerveModuleMDK(
   blDriveID,
   blTurnID,
   blCancoderID,
   turnMotorZeroPos[2],
   invertDL,
   invertTL,
   invertEncDL,
   invertEncTL
   );

// back right swerve module      
 modules[3] = new SwerveModuleMDK(
   brDriveID,
   brTurnID,
   brCancoderID,
   turnMotorZeroPos[3],
   invertDR,
   invertTR,
   invertEncDR,
   invertEncTR
   );

 return modules;
}

public void updatePID(SwerveModule[] modules) {
// update modules (the rotate PID is updated in SwerveDriveSystem)
  int i=0;
  while (i<4){
    modules[i].setTurnPID();
    modules[i].setDrivePID();
    i++;
  }
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