// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Utilities.Util;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants{

// Neo    
public static class Constants_Swerve{
// Robot Dimensions for Neo Swerve
    public static  double  wheelDiamInches = 3.125;//0.079375 meters 
    public static double wheelCircumferenceMaters=wheelDiamInches*Math.PI*0.0254; // 0.24936
    private static double gearRatioDrive=5.25; 
    public static double MPSToRPM = 60.0*gearRatioDrive/wheelCircumferenceMaters;  // 1,263.233
    //  maximum velocity called for when stick input is 1
    public static  double  maxVelocityFPS = 14.8;  //max speed in feet/sec
    // scales the stick -1 to 1 input to meters/sec
    public static double maxVelocityMPS = 0.3048*maxVelocityFPS;      
    public static double maxDriveRPM = maxVelocityMPS*MPSToRPM;

    // positive x: forward     positive y: left  
    //FL-FR-BL-BR
    public static  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(0.4064, 0.4064), 
        new Translation2d(0.4064, -0.4064), 
        new Translation2d(-0.4064, 0.4064), 
        new Translation2d(-0.4064, -0.4064)
        );


            // Motor Can ID's    
    public static  int flDriveID=2;
    public static int flTurnID=3;
    public static int frDriveID=4;
    public static int frTurnID=5;
    public static int blDriveID=10;
    public static int blTurnID=1;
    public static int brDriveID=6;
    public static int brTurnID=7;
    // Turn Motor Offsets
    public static double[] turnMotorZeroPos={0,0,0,0};
    // Invert  Motors
    public static boolean invertDL=false;
    public static boolean invertDR=false;
    public static boolean invertTL=true;
    public static boolean invertTR=true;
    // Invert  Drive Encoders
    public static boolean invertEncDL=false;
    public static boolean invertEncDR=false;
    // Invert Turn Encoders
    public static boolean invertEncTL=false;
    public static boolean invertEncTR=false;
//  PID Constants
    // Drive Motor Constants
    public static double kP_drive=0.0;  //0.00005;
    public static double kF_drive=0.000175;
    public static double kS_drive = 0;
    public static double kV_drive = 0;
    public static double kA_drive = 0;
// Use these when running turn motors in Smart Motion Mode 
// kF_turn=0.00598;   kP_turn=0.01;
// use these when running turn motors in position control kP=5.8,kF=0
//  Turn (Swerve) Motor Constants
    public static double kP_turn=5.8;
    public static double kD_turn=0.2;
    public static double kF_turn=0.0;   
    // Rotate (omega) Constants
    public static double kP_rotate=4.0;
    public static double kD_rotate=0.2;
    public static double AllowErr_rotate=0.01;
    public static double TrapMaxVel_rotate=20;
    public static double TrapMaxAcc_rotate=10;
    public static double rotateDPS=225;
    public static double rotateRP20msec=rotateDPS*Math.PI/(50.0*180.0);
    // these are just for compat. with MK4
    public static double SMMaxVel_turn=0;
    public static double SMMaxAcc_turn=0;
    public static double MPSToNativeSpeed = 0;
    public static double RadiansToNativePos = 0;
    public static double kP_driveff=0;
    public static double SMMaxVelRadPerSec_turn = 0;
    public static double  SMMaxAccRadPerSec2_turn = 0;

}

// MK4
public static class Constants_Swerve2{
    // Robot Dimensions for MK4 Swerve
    public static  double  wheelDiamInches = 3.95;//0.10033 meters 
    public static double wheelCircumferenceMaters=wheelDiamInches*Math.PI*0.0254; //0.315195
    private static double gearRatioDrive=8.1428; 
    public static double MPSToRPM = 60.0*gearRatioDrive/wheelCircumferenceMaters;  // 1,550.0499
    public static double MPSToNativeSpeed = MPSToRPM*2048.0/600.0;  // convert m/sec to Talon speed unit (counts/100ms)
    public static  double  maxVelocityFPS = 14.2;  //max speed in feet/sec
    //scales the stick -1 to 1 input to meters/sec
    public static double maxVelocityMPS = 0.3048*maxVelocityFPS; // 4.328     
    public static double maxDriveRPM = maxVelocityMPS*MPSToRPM; //  6708
    public static double maxOmega = 3.14; // rad/sec
    public static double RadiansToNativePos=4096.0/(2*Math.PI);



 // positive x: forward     positive y: left  
 // FR-FL-BR-BL
    public static  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(0.4064, -0.4064), 
    new Translation2d(0.4064, +0.4064), 
    new Translation2d(-0.4064, -0.4064), 
    new Translation2d(-0.4064, +0.4064));

        // Motor Can ID's    
        public static  int frDriveID=1;
        public static int frTurnID=2;
        public static int flDriveID=3;
        public static int flTurnID=4;
        public static int brDriveID=5;
        public static int brTurnID=6;
        public static int blDriveID=7;
        public static int blTurnID= 8;
    
    // Turn Module Offsets in degrees   FR-FL-BR-BL
    public static double[] turnMotorZeroPos={-15.6, 21.4, 12.4, -168.6};

    // Invert  Motors`
    public static boolean invertDL=false;
    public static boolean invertDR=false;
    public static boolean invertTL=false;
    public static boolean invertTR=false;
    // Invert  Drive Encoders
    public static boolean invertEncDL=false;
    public static boolean invertEncDR=false;
    // Invert Turn Encoders
    public static boolean invertEncTL=false;
    public static boolean invertEncTR=false;
    //  PID Constants
    // Drive Motor Constants
    public static double kP_drive=0;  //1.19 from characterization;
    public static double kF_drive=0.04952;   // 1023/20660

    public static double kA_drive=0.105;  // V*sec^2/m  (calculated from 0.0373 *(1/3600*GR)
    public static double kV_drive=2.78;  // V*sec/motorRPM , 0.868 from characterization * 1/(60*GR)
    public static double kS_drive=0.510;  // Volts  0.539 from characterization
    public static double kP_driveff=0.002209;  // V*sec/motorRPM,   1.06 from characterization * 1/(60*GR)

//  Turn (Swerve) Motor Constants
    public static double kP_turn=0.5; //0.5,   0.421 from characterization 
    public static double kD_turn=0;  // 0 from characterization    
    public static double kF_turn=0.0;   //  max turn RPM on ground = 485, 3310 units/100ms
 //   public static double kF_turn=0.0;   //  max turn RPM on ground = 485, 3310 units/100ms

    public static double kA_turn=0.407;  // 0.407 from characterization   
    public static double kV_turn=15.7;   // 15.7 from characterization
    public static double kS_turn=0.611;  // 0.611 from characterization
    public static double SMMaxVelRadPerSec_turn=60;  
    public static double SMMaxAccRadPerSec2_turn=60;
    public static double SMMaxVel_turn= 0.1*4096*SMMaxVelRadPerSec_turn/(2*Math.PI);
    public static double SMMaxAcc_turn=0.1*4096*SMMaxAccRadPerSec2_turn/(2*Math.PI);


    // Rotate (omega) Constants
    public static double kP_rotate=4;
    public static double kD_rotate=0.0;
    public static double AllowErr_rotate=0.01;
    public static double TrapMaxVel_rotate=20;
    public static double TrapMaxAcc_rotate=10;
    public static double rotateDPS=225;
    public static double rotateRP20msec=rotateDPS*Math.PI/(50.0*180.0);

}

        // Cancoder ID's
        public static  int frCancoderID=9;  
        public static  int flCancoderID=10;
        public static  int brCancoderID=11;
        public static  int blCancoderID=12;


    // max and min motor outputs
    public static double kMaxOutputDrive=1;
    public static double kMinOutputDrive=-1;
    public static double kMaxOutputTurn=0.8;
    public static double kMinOutputTurn=-0.8;



    // Controller Constants

    // which stick am I using, 0 =  white controller, 1 - switch 
    public static int stickNum=1;
    public static int stickPort=1;

//  Button Numbers
// X on white controller, btn 3
// Y on switch controller, btn 1
    public static int[] btn_resetencoder={3,1};  

// A on white controller, btn 1
// B on switch controller, btn 2
    public static int[] btn_resetgyro={1,2};  


// B on white controller, btn 2
// A on switch controller, btn 3
    public static int[] btn_updatePID={2,3};

// Ltop Trig on white controller, btn 5
// Ltop Trig on switch controller, btn 5
    public static int[] btn_bumpCCW={5,5};

// Rtop Trig on white controller, btn 6
// Rtop Trig on switch controller, btn 6
    public static int[] btn_bumpCW={6,6};

 // left_small button on white controller, btn 7
 // - button on switch, btn 9
    public static int[] btn_followcam={7,9}; 

// right_small button on white controller, btn 8
// + button on switch, btn 10
    public static int[] btn_turbo={8,10}; 

    public static int pov_rotate0=0;  // 
    public static int pov_rotate90=270;  // 
    public static int pov_rotate180=180;  //
    public static int pov_rotate270=90;  // 


    // AR following PID constants
    public static double ar_kp_vx = 0;
    public static double ar_ki_vx = 0;
    public static double ar_kp_vy = .005;
    public static double ar_ki_vy = 0;
    public static double ar_kp_hs = 0;
    public static double ar_ki_hs = 0;
    



// copied from Swerve Controller Example
    public static final class AutoConstants {
//        public static final double kMaxSpeedMetersPerSecond = 3;
//        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
 

}