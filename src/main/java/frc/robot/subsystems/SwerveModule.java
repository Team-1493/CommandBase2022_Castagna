// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;
import static frc.robot.Constants.Constants_Swerve.*;


import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class SwerveModule{
    private final CANSparkMax m_drive;
    private final SparkMaxPIDController c_drive;
    private final RelativeEncoder e_drive;

    private final CANSparkMax m_turn;
    private final SparkMaxPIDController c_turn;
    private final RelativeEncoder e_turn;

    private final String driveVel=this.getClass().getName()+"_drive_vel";
    private final String drivePos=this.getClass().getName()+"_drive_pos";
    private final String turnVel=this.getClass().getName()+"_turn_vel";
    private final String turnPos=this.getClass().getName()+"_turn_pos";
    int rev=1;

    
public SwerveModule(int driveID, int turnID, boolean invD, boolean invT,
                 boolean invEncD, boolean invEncT){

                        
// set up the drive motor                                 
    m_drive=new CANSparkMax(driveID,MotorType.kBrushless);
    m_drive.restoreFactoryDefaults();
    m_drive.setInverted(invD);

// set up the drive PID controller    
    c_drive=m_drive.getPIDController();

    c_drive.setP(kP_drive);
    c_drive.setFF(kF_drive);      

// set up the drive encoder
    e_drive=m_drive.getEncoder();

// set up the turn motor    
    m_turn=new CANSparkMax(turnID,MotorType.kBrushless);
    m_turn.restoreFactoryDefaults();
    m_turn.setInverted(invT);
    m_turn.setIdleMode(IdleMode.kBrake);

// set up the turn encoder
    e_turn=m_turn.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,4096);
    e_turn.setInverted(invEncT);  
    e_turn.setPosition(0.0);
    System.out.println("e_tern cpr = "+e_turn.getCountsPerRevolution());    
// set up the turn PID controller. It will run the motor in SmartMotion mode,
// so we set up max/min velocity and max acceleration allowed.
    c_turn=m_turn.getPIDController();                  
    c_turn.setP(kP_turn);
    c_turn.setD(kD_turn);
    c_turn.setFF(kF_turn);     
    c_turn.setFeedbackDevice(e_turn);
    c_turn.setOutputRange(kMinOutputTurn,kMaxOutputTurn);
    c_turn.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,0);
}


// get the module state for odometry calculations: velocity in mps and module angle in radians
public SwerveModuleState getState() {
    return new SwerveModuleState(
        e_drive.getVelocity()/MPSToRPM, new Rotation2d(2*Math.PI*e_turn.getPosition()));
  }


public void setMotors(double speed,double turnAngle) {
    c_drive.setReference(speed*MPSToRPM, ControlType.kVelocity);
    c_turn.setReference(turnAngle/(2*Math.PI), ControlType.kPosition);
//    c_turn.setReference(revolutions, ControlType.kSmartMotion); 
  }


  public void writeEncoders() {
    SmartDashboard.putNumber(driveVel, e_drive.getVelocity());
    SmartDashboard.putNumber(drivePos, e_drive.getPosition());
    SmartDashboard.putNumber(turnVel, e_turn.getVelocity());
    SmartDashboard.putNumber(turnPos, e_turn.getPosition());
    
  }

  public void resetEncoders() {
      e_drive.setPosition(0);
//      e_turn.setPosition(0);
     }

// get the turn encoder position, measured in rotations
     public double getTurnPosition() {
        return e_turn.getPosition();
       }

    // get the turn encoder position, measured in rotations
     public double getTurnPosition_Rad() {
        return 2*Math.PI*e_turn.getPosition();
       }

// dummy method to be consistent with MK4 module
       public double getTurnAbsPosition() {
        return 0.0;
       }       


// get the turn velocity velocity, measured in rotations
public double getTurnVelocity() {
    return e_turn.getVelocity();
   }       

// get the turn encoder position, measured in rotations
public double getTurnMotorOutput() {
    return m_turn.getAppliedOutput();
   }       


// get the drive encoder position, measured in rotations
     public double getDrivePosition() {
        return e_drive.getPosition();
       }

// get the drive encoder velocity, measured in rpm
     public double getDriveVelocity() {
        return e_drive.getVelocity();
       }

  
    public void setTurnPID() {
        c_turn.setP(kP_turn);
        c_turn.setD(kD_turn);
        c_turn.setOutputRange(kMinOutputTurn,kMaxOutputTurn);
        c_turn.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,0);                        
       }

    public void setDrivePID() {
        c_drive.setP(kP_drive);
        c_drive.setFF(kF_drive);
       }       


}
