package frc.robot.Utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSystem;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Constants_Swerve.*;

public class DriverStationInterface{
  
public DriverStationInterface(SwerveDriveSystem sds){
    SmartDashboard.putNumber("kP_Drive",kP_drive);
    SmartDashboard.putNumber("kF_Drive",kF_drive);
    SmartDashboard.putNumber("kP_driveff",kP_driveff);
    SmartDashboard.putNumber("kP_Turn",kP_turn);
    SmartDashboard.putNumber("kD_Turn",kD_turn);
    SmartDashboard.putNumber("kF_Turn",kF_turn);
    SmartDashboard.putNumber("SMMaxVelRadPerSec_turn",SMMaxVelRadPerSec_turn);
    SmartDashboard.putNumber("SMMaxAccRadPerSec2_turn",SMMaxAccRadPerSec2_turn);
    SmartDashboard.putNumber("MaxOutput Turn",kMaxOutputTurn);
    
    SmartDashboard.putNumber("kP_Rotate",kP_rotate);                    
    SmartDashboard.putNumber("kD_Rotate",kD_rotate);                   
    SmartDashboard.putNumber("kD_Rotate",kD_rotate);                    
    SmartDashboard.putNumber("AllErr_Rotate",AllowErr_rotate);                    
    SmartDashboard.putNumber("TrapMaxVel_Rotate",TrapMaxVel_rotate);
    SmartDashboard.putNumber("TrapMaxAcc_Rotate",TrapMaxAcc_rotate);
    SmartDashboard.putNumber("rotateDPS",rotateDPS);
    SmartDashboard.putNumber("rotateRP20msec",rotateRP20msec);
  
    SmartDashboard.putNumber("Max Vel FPS",maxVelocityFPS);
    SmartDashboard.putNumber("Max Drive RPM",maxDriveRPM);
    
    }

    public static void getNewConstants(){
        // Read new constants from the dashboard and store in Constants class
        kP_turn= SmartDashboard.getNumber("kP_Turn",kP_turn);
        kD_turn= SmartDashboard.getNumber("kD_Turn",kD_turn);
        kF_turn= SmartDashboard.getNumber("kF_Turn",kF_turn);
        SMMaxVelRadPerSec_turn= SmartDashboard.getNumber("SMMaxVelRadPerSec_turn",
             SMMaxVelRadPerSec_turn);
        SMMaxAccRadPerSec2_turn= SmartDashboard.getNumber("SMMaxAccRadPerSec2_turn",
            SMMaxAccRadPerSec2_turn);
        SMMaxVel_turn= 0.1*4096*SMMaxVelRadPerSec_turn/(2*Math.PI);
        SMMaxAcc_turn=0.1*4096*SMMaxAccRadPerSec2_turn/(2*Math.PI);
        kMaxOutputTurn = SmartDashboard.getNumber("MaxOutput Turn",kMaxOutputTurn);
        kMinOutputTurn = -SmartDashboard.getNumber("MaxOutput Turn",kMaxOutputTurn);
      
      
        kP_drive= SmartDashboard.getNumber("kP_Drive",kP_drive);
        kF_drive= SmartDashboard.getNumber("kF_Drive",kF_drive);
      
        
        kP_rotate= SmartDashboard.getNumber("kP_Rotate",kP_rotate);
        kD_rotate= SmartDashboard.getNumber("kD_Rotate",kD_rotate);
        AllowErr_rotate= SmartDashboard.getNumber("AllErr_Rotate",AllowErr_rotate);
        TrapMaxVel_rotate= SmartDashboard.getNumber("TrapMaxVel_Rotate",TrapMaxVel_rotate);
        TrapMaxAcc_rotate= SmartDashboard.getNumber("TrapMaxAcc_Rotate",TrapMaxAcc_rotate);
        rotateDPS= SmartDashboard.getNumber("rotateDPS",rotateDPS);
        rotateRP20msec= rotateDPS*Math.PI/(50.0*180.0);
        
        maxVelocityFPS= SmartDashboard.getNumber("Max Vel FPS",maxVelocityFPS);
      
        // Calculated Values
        maxVelocityMPS = 0.3048*maxVelocityFPS; 
        maxDriveRPM=maxVelocityMPS*MPSToRPM;
        SmartDashboard.putNumber("Max Drive RPM",maxDriveRPM);
      
      }
      
      

}