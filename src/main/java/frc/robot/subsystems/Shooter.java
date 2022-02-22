// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Shooter extends SubsystemBase {

  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTable limelight= inst.getTable("limelight");
  public NetworkTableEntry tyEntry = limelight.getEntry("ty");
  public NetworkTableEntry tvEntry = limelight.getEntry("tv");


  TalonFX shooterR = new TalonFX(13);
  TalonFX shooterL = new TalonFX(12);
  double shooterKs=0.024,shooterKv=0.000147,shooterKa=0.1;
  double shooterKp=0.2;
  
  double currentShooterSpeedL=0; 
  double currentShooterSpeedR=0; 
  double speedFactor=100;
  double shooterSpeedHigh=0;
  double shooterSpeedLow=850;
  double shooterSpeedManual=1750;
  double shooterSpeed=0;
  public boolean atSpeed=false;

  SimpleMotorFeedforward shootFeedforward = 
    new SimpleMotorFeedforward(shooterKs,shooterKv,shooterKa);

public Shooter(){
  
    shooterL.configFactoryDefault();
    shooterL.setNeutralMode(NeutralMode.Coast);
    shooterL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 25);
    shooterL.config_kP(0, shooterKp);

    shooterR.configFactoryDefault();    
    shooterR.setNeutralMode(NeutralMode.Coast);
    shooterR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 25);
    shooterR.config_kP(0, shooterKp);

    SmartDashboard.putNumber("shooterL Vel",0);
    SmartDashboard.putNumber("shooterR Vel",0);
    SmartDashboard.putNumber("shooter kS",shooterKs);
    SmartDashboard.putNumber("shooter kV",shooterKv);
    SmartDashboard.putNumber("shooter kA",shooterKa);
    SmartDashboard.putNumber("shooter kP",shooterKp);
    SmartDashboard.putNumber("Manual Shoot Speed",shooterSpeedManual);
    SmartDashboard.putBoolean("Shooter At Spoeed",atSpeed);



  }


public void shootHigh(){
    if(tvEntry.getDouble(1)==1){
      double ty=tyEntry.getDouble(1);
      shooterSpeed=speedFactor;
    }
    else shooterSpeed=0;
    set();
}


public void shootLow(){
  shooterSpeed=shooterSpeedLow;
  set();
}


public void shootManual(){
  shooterSpeedManual=SmartDashboard.getNumber("Manual Shoot Speed", shooterSpeedManual);
  shooterSpeed=shooterSpeedManual;
  set();
}

public void set(){
  shooterL.set(ControlMode.Velocity, shooterSpeed*2048/600, DemandType.ArbitraryFeedForward ,
       shootFeedforward.calculate(shooterSpeed));
  shooterR.set(ControlMode.Velocity, -shooterSpeed*2048/600, DemandType.ArbitraryFeedForward ,
       shootFeedforward.calculate(-shooterSpeed)); 
}


public void stopShooter(){
      shooterSpeed=0;
      shooterL.set(ControlMode.PercentOutput,0);
      shooterR.set(ControlMode.PercentOutput,0);
      atSpeed=false;
    }



    public void updateConstants(){
      shooterKs=SmartDashboard.getNumber("shooter kS", shooterKs);
      shooterKv=SmartDashboard.getNumber("shooter kV", shooterKv);
      shooterKa=SmartDashboard.getNumber("shooter kA", shooterKa);
      shooterKp=SmartDashboard.getNumber("shooter kP", shooterKp);
      shootFeedforward=new SimpleMotorFeedforward(shooterKs,shooterKv,shooterKa);
      shooterL.config_kP(0, shooterKp);
      shooterR.config_kP(0, shooterKp);
    }



    @Override
    public void periodic() {
      currentShooterSpeedL=shooterL.getSelectedSensorVelocity()*600/2048;
      currentShooterSpeedR=shooterR.getSelectedSensorVelocity()*600/2048;
      if (shooterSpeed>0 && Math.abs(currentShooterSpeedL-shooterSpeed)<30 && 
                    Math.abs(-currentShooterSpeedR-shooterSpeed)<30)
          atSpeed=true;
      else atSpeed=false;
      SmartDashboard.putNumber("Shooter set speed",shooterSpeed);  
      SmartDashboard.putBoolean("Shooter At Spoeed",atSpeed);
      SmartDashboard.putNumber("shooterL Vel", currentShooterSpeedL);
      SmartDashboard.putNumber("shooterR Vel", currentShooterSpeedR);

//      SmartDashboard.putNumber("shooterR Vel",currentShooterSpeedR);
    }
  

}
