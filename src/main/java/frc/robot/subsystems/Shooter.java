// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
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

  private Timer timer = new Timer();
  private double shooterTOTGoal=0.2;
  private double currentTimeOnTarget=0;
  private double startTime=0;

  double topKs=0.024,topKv=0.000147,topKa=0.1,topKp=0.2;
  double bottomKs=0.024,bottomKv=0.000141,bottomKa=0.1,bottomKp=0.090;
  
  double currentShooterSpeedL=0; 
  double currentShooterSpeedR=0; 
  double speedFactor=100;
  double shooterSpeedHigh=0;
  double shooterSpeedLow=850;
  double shooterSpeedManual=1750;
  double shooterSpeed=0;
  double bottomclpo=0.4;
  public boolean atSpeed=false;
  private double shooterTolerance=50;

  SimpleMotorFeedforward topFF = 
    new SimpleMotorFeedforward(topKs,topKv,topKa);

    SimpleMotorFeedforward bottomFF = 
    new SimpleMotorFeedforward(bottomKs,bottomKv,bottomKa);

public Shooter(){
  
    timer.start();

    shooterL.configFactoryDefault();
    shooterL.setNeutralMode(NeutralMode.Coast);
    shooterL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 25);
    shooterL.config_kP(0, topKp);

    shooterR.configFactoryDefault();    
    shooterR.setNeutralMode(NeutralMode.Coast);
    shooterR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 25);
    shooterR.configClosedLoopPeakOutput(0, bottomclpo);
    shooterR.config_kP(0, bottomKp);

//    shooterR.configClosedloopRamp(0.25);

    SmartDashboard.putNumber("shooterR clpo", bottomclpo);
    SmartDashboard.putNumber("shooterL Vel",0);
    SmartDashboard.putNumber("shooterR Vel",0);
    SmartDashboard.putNumber("shooter top kS",topKs);
    SmartDashboard.putNumber("shooter top kV",topKv);
    SmartDashboard.putNumber("shooter top kA",topKa);
    SmartDashboard.putNumber("shooter top kP",topKp);
    SmartDashboard.putNumber("shooter bottom kS",bottomKs);
    SmartDashboard.putNumber("shooter bottom kV",bottomKv);
    SmartDashboard.putNumber("shooter bottom kA",bottomKa);
    SmartDashboard.putNumber("shooter bottom kP",bottomKp);
    SmartDashboard.putNumber("Manual Shoot Speed",shooterSpeedManual);
    SmartDashboard.putBoolean("Shooter At Spoeed",atSpeed);
    SmartDashboard.putNumber("Shooter At Spoeed",shooterTolerance);

  

  }


public void shootHigh(){
    if(tvEntry.getDouble(1)==1){
      double ty=tyEntry.getDouble(1);
      shooterSpeed=2102.42876*Math.pow(ty, -0.06006);
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
  double ffr = -shooterSpeed*(shooterSpeed* (2.1657*Math.pow(10,-8)) +8.4057*Math.pow(10,-5))   -0.04;
  shooterL.set(ControlMode.Velocity, shooterSpeed*2048/600, DemandType.ArbitraryFeedForward ,
       topFF.calculate(shooterSpeed));

       shooterR.set(ControlMode.Velocity, -shooterSpeed*2048/600, DemandType.ArbitraryFeedForward ,
       bottomFF.calculate(-shooterSpeed));

//  shooterR.set(ControlMode.Velocity, -shooterSpeed*2048/600, DemandType.ArbitraryFeedForward ,ffr); 
}


public void stopShooter(){
      shooterSpeed=0;
      shooterL.set(ControlMode.PercentOutput,0);
      shooterR.set(ControlMode.PercentOutput,0);
      atSpeed=false;
    }



    public void updateConstants(){
      shooterTolerance=SmartDashboard.getNumber("shooter tolerance", shooterTolerance);
      shooterTolerance=SmartDashboard.getNumber("shooter TOTgoal", shooterTOTGoal);

      topKs=SmartDashboard.getNumber("shooter top kS", topKs);
      topKv=SmartDashboard.getNumber("shooter top kV", topKv);
      topKa=SmartDashboard.getNumber("shooter top kA", topKa);
      topKp=SmartDashboard.getNumber("shooter top kP", topKp);
      topFF=new SimpleMotorFeedforward(topKs,topKv,topKa);
      shooterL.config_kP(0, topKp);
      
      bottomKs=SmartDashboard.getNumber("shooter bottom kS", bottomKs);
      bottomKv=SmartDashboard.getNumber("shooter bottom kV", bottomKv);
      bottomKa=SmartDashboard.getNumber("shooter bottom kA", bottomKa);
      bottomKp=SmartDashboard.getNumber("shooter bottom kP", bottomKp);
      bottomclpo=SmartDashboard.getNumber("shooterR clpo", bottomclpo);
      bottomFF=new SimpleMotorFeedforward(bottomKs,bottomKv,bottomKa);     
      shooterR.config_kP(0, bottomKp);
      shooterR.configClosedLoopPeakOutput(0, bottomclpo);
    }



    @Override
    public void periodic() {
      currentShooterSpeedL=shooterL.getSelectedSensorVelocity()*600/2048;
      currentShooterSpeedR=shooterR.getSelectedSensorVelocity()*600/2048;

      if (shooterSpeed>0 && Math.abs(currentShooterSpeedL-shooterSpeed)<shooterTolerance && 
                    Math.abs(-currentShooterSpeedR-shooterSpeed)<shooterTolerance){
        currentTimeOnTarget=timer.get()-startTime;
        if(currentTimeOnTarget>shooterTOTGoal) atSpeed=true;
      }
     else {
        startTime=timer.get();
        atSpeed=false;
      } 
      SmartDashboard.putNumber("Shooter set speed",shooterSpeed);  
      SmartDashboard.putBoolean("Shooter At Spoeed",atSpeed);
      SmartDashboard.putNumber("shooterL Vel",currentShooterSpeedL);
      SmartDashboard.putNumber("shooterR Vel",currentShooterSpeedR);
      }
}
