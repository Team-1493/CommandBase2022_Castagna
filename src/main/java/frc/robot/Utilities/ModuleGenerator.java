package frc.robot.Utilities;
import frc.robot.subsystems.SwerveModuleMDK;

public class ModuleGenerator{
  
  private int frCancoderID=11, flCancoderID=13, brCancoderID=15, blCancoderID=17;
  private int frDriveID=1, frTurnID=2;
  private int flDriveID=3, flTurnID=4;
  private int brDriveID=5,  brTurnID=6;
  private int blDriveID=7,  blTurnID= 8;
  private boolean invertDriveMotors = false;
  private boolean invertTurnMotors = false;
  private boolean invertDriveEncoders=false;
  private boolean invertTurnEncoders=false;
  
  // Turn Module Offsets in degrees   FR-FL-BR-BL
  public static double[] turnMotorZeroPos={64.7, 105.9, 29.9, 56.9};


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
   invertDriveMotors,
   invertTurnMotors,
   invertDriveEncoders,
   invertTurnEncoders
   );

// front right swerve module     
   modules[1] = new SwerveModuleMDK(
   flDriveID,
   flTurnID,
   flCancoderID,
   turnMotorZeroPos[1],
   invertDriveMotors,
   invertTurnMotors,
   invertDriveEncoders,
   invertTurnEncoders
      );

// back left swerve module         
 modules[2] = new SwerveModuleMDK(
   brDriveID,
   brTurnID,
   brCancoderID,
   turnMotorZeroPos[2],
   invertDriveMotors,
   invertTurnMotors,
   invertDriveEncoders,
   invertTurnEncoders
   );

// back right swerve module      
 modules[3] = new SwerveModuleMDK(
   blDriveID,
   blTurnID,
   blCancoderID,
   turnMotorZeroPos[3],
   invertDriveMotors,
   invertTurnMotors,
   invertDriveEncoders,
   invertTurnEncoders
   );

 return modules;
}

}