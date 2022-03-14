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


public class Shooter extends SubsystemBase {

  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTable limelight= inst.getTable("limelight");
  public NetworkTableEntry tyEntry = limelight.getEntry("ty");
  public NetworkTableEntry tvEntry = limelight.getEntry("tv");


  TalonFX shooterGrey = new TalonFX(13);
  TalonFX shooterBlue = new TalonFX(12);

  private Timer timer = new Timer();
  private double shooterTOTGoal=0.25;
  private double currentTimeOnTarget=0;
  private double startTime=0;

  double blueKs=0.015,blueKv=0.000158,blueKa=0.1,blueKp=0.25,blueKd=8.0;
  double GreyKs=0.022,GreyKv=0.000155,GreyKa=0.1,GreyKp=0.25,GreyKd=8.0;
  
  double currentShooterSpeedBlue=0; 
  double currentShooterSpeedGrey=0; 
  double speedFactor=100;
  double shooterSpeedHigh=0;
  double shooterSpeedLow=850;
  double shooterSpeedManual=1750;
  double shooterSpeed=0;
  double Greyclpo=0.4;
  public boolean atSpeed=false;
  private double shooterTolerance=35;

  SimpleMotorFeedforward blueFF = 
    new SimpleMotorFeedforward(blueKs,blueKv,blueKa);

    SimpleMotorFeedforward GreyFF = 
    new SimpleMotorFeedforward(GreyKs,GreyKv,GreyKa);

public Shooter(){
  
    timer.start();

    shooterBlue.configFactoryDefault();
    shooterBlue.setNeutralMode(NeutralMode.Coast);
    shooterBlue.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 25);
    shooterBlue.config_kP(0, blueKp);
    shooterBlue.configVoltageCompSaturation(11.5);

    shooterGrey.configFactoryDefault();    
    shooterGrey.setNeutralMode(NeutralMode.Coast);
    shooterGrey.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 25);
    shooterGrey.configClosedLoopPeakOutput(0, Greyclpo);
    shooterGrey.config_kP(0, GreyKp);
    shooterBlue.configVoltageCompSaturation(11.5);

//    shooterGrey.configClosedloopRamp(0.25);

    SmartDashboard.putNumber("shooterGrey clpo", Greyclpo);
    SmartDashboard.putNumber("shooterBlue Vel",0);
    SmartDashboard.putNumber("shooterGrey Vel",0);
    SmartDashboard.putNumber("shooter blue kS",blueKs);
    SmartDashboard.putNumber("shooter blue kV",blueKv);
    SmartDashboard.putNumber("shooter blue kA",blueKa);
    SmartDashboard.putNumber("shooter blue kP",blueKp);
    SmartDashboard.putNumber("shooter blue kD",blueKd);
    SmartDashboard.putNumber("shooter Grey kS",GreyKs);
    SmartDashboard.putNumber("shooter Grey kV",GreyKv);
    SmartDashboard.putNumber("shooter Grey kA",GreyKa);
    SmartDashboard.putNumber("shooter Grey kP",GreyKp);
    SmartDashboard.putNumber("shooter Grey kD",GreyKd);
    SmartDashboard.putNumber("Manual Shoot Speed",shooterSpeedManual);
    SmartDashboard.putBoolean("Shooter At Spoeed",atSpeed);
    SmartDashboard.putNumber("shooter tolerance",shooterTolerance);
    SmartDashboard.putNumber("shooter TOTgoal", shooterTOTGoal);
  }


public void shootHigh(){
    if(tvEntry.getDouble(1)==1){
      double ty=tyEntry.getDouble(1);
      double x=1/Math.tan(ty*Math.PI/180.);

      shooterSpeed=0.96*(-1.8312*x*x+74.388*x+1504.1);
 // shooterSpeed=-5.585*x*x+227.92*x+1057.5;
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


public void shootAtSpeed(int rpm){
  shooterSpeed=rpm;
  set();
}

public void set(){
  shooterBlue.set(ControlMode.Velocity, shooterSpeed*2048/600, DemandType.ArbitraryFeedForward ,
       blueFF.calculate(shooterSpeed));

       shooterGrey.set(ControlMode.Velocity, -shooterSpeed*2048/600, DemandType.ArbitraryFeedForward ,
       GreyFF.calculate(-shooterSpeed));

//  shooterGrey.set(ControlMode.Velocity, -shooterSpeed*2048/600, DemandType.ArbitraryFeedForward ,ffr); 
}


public void stopShooter(){
      shooterSpeed=0;
      shooterBlue.set(ControlMode.PercentOutput,0);
      shooterGrey.set(ControlMode.PercentOutput,0);
      atSpeed=false;
    }



    public void updateConstants(){
      shooterTolerance=SmartDashboard.getNumber("shooter tolerance", shooterTolerance);
      shooterTOTGoal=SmartDashboard.getNumber("shooter TOTgoal", shooterTOTGoal);

      blueKs=SmartDashboard.getNumber("shooter blue kS", blueKs);
      blueKv=SmartDashboard.getNumber("shooter blue kV", blueKv);
      blueKa=SmartDashboard.getNumber("shooter blue kA", blueKa);
      blueKp=SmartDashboard.getNumber("shooter blue kP", blueKp);
      blueKd=SmartDashboard.getNumber("shooter blue kD", blueKd);
      blueFF=new SimpleMotorFeedforward(blueKs,blueKv,blueKa);
      shooterBlue.config_kP(0, blueKp);
      shooterBlue.config_kD(0, blueKd);
      
      GreyKs=SmartDashboard.getNumber("shooter Grey kS", GreyKs);
      GreyKv=SmartDashboard.getNumber("shooter Grey kV", GreyKv);
      GreyKa=SmartDashboard.getNumber("shooter Grey kA", GreyKa);
      GreyKp=SmartDashboard.getNumber("shooter Grey kP", GreyKp);
      GreyKd=SmartDashboard.getNumber("shooter Grey kD", GreyKd);
      Greyclpo=SmartDashboard.getNumber("shooterGrey clpo", Greyclpo);
      GreyFF=new SimpleMotorFeedforward(GreyKs,GreyKv,GreyKa);     
      shooterGrey.config_kP(0, GreyKp);
      shooterGrey.config_kD(0, GreyKd);
      shooterGrey.configClosedLoopPeakOutput(0, Greyclpo);
    }



    @Override
    public void periodic() {
      currentShooterSpeedBlue=shooterBlue.getSelectedSensorVelocity()*600/2048;
      currentShooterSpeedGrey=shooterGrey.getSelectedSensorVelocity()*600/2048;

      if (shooterSpeed>0 && Math.abs(currentShooterSpeedBlue-shooterSpeed)<shooterTolerance && 
                    Math.abs(-currentShooterSpeedGrey-shooterSpeed)<shooterTolerance){
        currentTimeOnTarget=timer.get()-startTime;
        if(currentTimeOnTarget>shooterTOTGoal) atSpeed=true;
      }
     else {
        startTime=timer.get();
        atSpeed=false;
      } 
      SmartDashboard.putNumber("Shooter set speed",shooterSpeed);  
      SmartDashboard.putBoolean("Shooter At Spoeed",atSpeed);
      SmartDashboard.putNumber("shooterBlue Vel",currentShooterSpeedBlue);
      SmartDashboard.putNumber("shooterGrey Vel",currentShooterSpeedGrey);
      }
}
