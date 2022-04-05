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
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class Shooter extends SubsystemBase {

  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTable limelight= inst.getTable("limelight");
  public NetworkTableEntry tyEntry = limelight.getEntry("ty");
  public NetworkTableEntry tvEntry = limelight.getEntry("tv");


  TalonFX shooterGrey = new TalonFX(13);
  TalonFX shooterBlue = new TalonFX(12);

  private Timer timer = new Timer();
  private double shooterTOTGoal=0.1;
  private double currentTimeOnTarget=0;
  private double startTime=0;
  private double blueKp=0.3,GreyKp=0.3;
  private double blueKd=7,GreyKd=7;
  private double blueKff=0.05,GreyKff=0.05;
  private double bluePeakFor=1,bluePeakRev=-1;

  private double greyPeakFor=1,greyPeakRev=-1;
  private double voltageComp=11.5;
  double currentShooterSpeedBlue=0; 
  double currentShooterSpeedGrey=0; 
  double speedFactor=100;
  double shooterSpeedHigh=0;
  double shooterSpeedLow=850;
  double shooterSpeedManual=1750;
  double shooterSpeed=0;
  
  public boolean atSpeed=false;
  private double shooterTolerance=20;


public Shooter(){
  
    timer.start();

    shooterBlue.configFactoryDefault();
    shooterBlue.setNeutralMode(NeutralMode.Coast);
    shooterBlue.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 25);
    shooterBlue.config_kP(0, blueKp);
    shooterBlue.config_kD(0, blueKd);
    shooterBlue.config_kF(0, blueKff);
    //  Data frame for integrated sensor position and velocity, default time is >109ms
    shooterBlue.setStatusFramePeriod(2, 20);

    shooterBlue.setStatusFramePeriod(4,251);
    shooterBlue.setStatusFramePeriod(8,241);
    shooterBlue.setStatusFramePeriod(10,239);
    shooterBlue.setStatusFramePeriod(12,233);
    shooterBlue.setStatusFramePeriod(14,229);

    shooterBlue.configVelocityMeasurementWindow(8, 25);
    shooterBlue.configPeakOutputForward(bluePeakFor);
    shooterBlue.configPeakOutputReverse(bluePeakRev);
    shooterBlue.configVoltageCompSaturation(voltageComp);


    shooterGrey.configFactoryDefault();    
    shooterGrey.setNeutralMode(NeutralMode.Coast);
    shooterGrey.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 25);
    shooterGrey.config_kP(0, GreyKp);
    shooterGrey.config_kD(0, GreyKd);
    shooterGrey.config_kF(0, GreyKff);
    //  Data frame for integrated sensor position and velocity, default time is >109ms
    shooterGrey.setStatusFramePeriod(2, 20);
    shooterGrey.setStatusFramePeriod(4, 251);
    shooterGrey.setStatusFramePeriod(8, 241);
    shooterGrey.setStatusFramePeriod(10, 239);
    shooterGrey.setStatusFramePeriod(12, 233);
    shooterGrey.setStatusFramePeriod(14, 229);
//    shooterGrey.setStatusFramePeriod(StatusFrame.Status_1_General, 211);

    
    shooterGrey.configVelocityMeasurementWindow(8, 25);
    shooterGrey.configPeakOutputForward(greyPeakFor);
    shooterGrey.configPeakOutputReverse(greyPeakRev);
    shooterGrey.configVoltageCompSaturation(voltageComp);

//    shooterGrey.configClosedloopRamp(0.25);

  SmartDashboard.putNumber("blue Peak For",bluePeakFor);
  SmartDashboard.putNumber("grey Peak For",greyPeakFor);

    SmartDashboard.putNumber("shooterBlue Vel",0);
    SmartDashboard.putNumber("shooterGrey Vel",0);
    SmartDashboard.putNumber("shooter blue kP",blueKp);
    SmartDashboard.putNumber("shooter Grey kP",GreyKp);
    SmartDashboard.putNumber("shooter blue kD",blueKd);
    SmartDashboard.putNumber("shooter Grey kD",GreyKd);
    SmartDashboard.putNumber("shooter blue kF",blueKff);
    SmartDashboard.putNumber("shooter Grey kF",GreyKff);
    SmartDashboard.putNumber("Manual Shoot Speed",shooterSpeedManual);
    SmartDashboard.putBoolean("Shooter At Spoeed",atSpeed);
    SmartDashboard.putNumber("shooter tolerance",shooterTolerance);
    SmartDashboard.putNumber("shooter TOTgoal", shooterTOTGoal);
  }


public void shootHigh(){
    if(tvEntry.getDouble(1)==1){
      double ty=tyEntry.getDouble(1);
      double x=1/(0.001+ty);

//      shooterSpeed=0.96*(-1.8312*x*x+74.388*x+1504.1); 
//      shooterSpeed = ty * 30;
      shooterSpeed=5293*x+1309;
      if (shooterSpeed>2700) shooterSpeed=2700;
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
       shooterBlue.set(ControlMode.Velocity, shooterSpeed*2048/600);
       shooterGrey.set(ControlMode.Velocity, shooterSpeed*2048/600);
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

      bluePeakFor=SmartDashboard.getNumber("blue Peak For", bluePeakFor);
      greyPeakFor=SmartDashboard.getNumber("grey Peak For", bluePeakFor);
      shooterBlue.configPeakOutputForward(bluePeakFor);
      shooterGrey.configPeakOutputForward(greyPeakFor);


      blueKp=SmartDashboard.getNumber("shooter blue kP", blueKp);
      shooterBlue.config_kP(0, blueKp);
      
      GreyKp=SmartDashboard.getNumber("shooter Grey kP", GreyKp);
      shooterGrey.config_kP(0, GreyKp);

      blueKd=SmartDashboard.getNumber("shooter blue kD", blueKd);
      shooterBlue.config_kD(0, blueKd);
      
      GreyKd=SmartDashboard.getNumber("shooter Grey kD", GreyKd);
      shooterGrey.config_kD(0, GreyKd);

      blueKff=SmartDashboard.getNumber("shooter blue kF", blueKff);
      shooterBlue.config_kF(0, blueKff);
      
      GreyKff=SmartDashboard.getNumber("shooter Grey kF", GreyKff);
      shooterGrey.config_kF(0, GreyKff);
    }



    @Override
    public void periodic() {
//      inst.flush();
      currentShooterSpeedBlue=shooterBlue.getSelectedSensorVelocity()*600/2048;
      currentShooterSpeedGrey=shooterGrey.getSelectedSensorVelocity()*600/2048;

      if (shooterSpeed>0 && (shooterSpeed-currentShooterSpeedBlue)<shooterTolerance && 
                    (shooterSpeed-currentShooterSpeedGrey)<shooterTolerance){
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
