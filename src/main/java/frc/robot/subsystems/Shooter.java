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


  TalonFX shooterGrey = new TalonFX(13);
  TalonFX shooterBlue = new TalonFX(12);

  private Timer timer = new Timer();
  private double shooterTOTGoal=0.25;
  private double currentTimeOnTarget=0;
  private double startTime=0;

  double blueKs=0.015,blueKv=0.000147,blueKa=0.1,blueKp=0.15;
  double GreyKs=0.022,GreyKv=0.000141,GreyKa=0.1,GreyKp=0.15;
  
  double currentShooterSpeedL=0; 
  double currentShooterSpeedR=0; 
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

    shooterGrey.configFactoryDefault();    
    shooterGrey.setNeutralMode(NeutralMode.Coast);
    shooterGrey.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 25);
    shooterGrey.configClosedLoopPeakOutput(0, Greyclpo);
    shooterGrey.config_kP(0, GreyKp);

//    shooterGrey.configClosedloopRamp(0.25);

    SmartDashboard.putNumber("shooterGrey clpo", Greyclpo);
    SmartDashboard.putNumber("shooterBlue Vel",0);
    SmartDashboard.putNumber("shooterGrey Vel",0);
    SmartDashboard.putNumber("shooter blue kS",blueKs);
    SmartDashboard.putNumber("shooter blue kV",blueKv);
    SmartDashboard.putNumber("shooter blue kA",blueKa);
    SmartDashboard.putNumber("shooter blue kP",blueKp);
    SmartDashboard.putNumber("shooter Grey kS",GreyKs);
    SmartDashboard.putNumber("shooter Grey kV",GreyKv);
    SmartDashboard.putNumber("shooter Grey kA",GreyKa);
    SmartDashboard.putNumber("shooter Grey kP",GreyKp);
    SmartDashboard.putNumber("Manual Shoot Speed",shooterSpeedManual);
    SmartDashboard.putBoolean("Shooter At Spoeed",atSpeed);
    SmartDashboard.putNumber("shooter tolerance",shooterTolerance);
    SmartDashboard.putNumber("shooter TOTgoal", shooterTOTGoal);

  

  }


public void shootHigh(){
    if(tvEntry.getDouble(1)==1){
      double ty=tyEntry.getDouble(1);
      double x=1/Math.tan(ty*Math.PI/180.);
      shooterSpeed=2250*Math.pow(ty, -0.075);

      shooterSpeed=-1.8312*x*x+74.388*x+1504.1;
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
      blueFF=new SimpleMotorFeedforward(blueKs,blueKv,blueKa);
      shooterBlue.config_kP(0, blueKp);
      
      GreyKs=SmartDashboard.getNumber("shooter Grey kS", GreyKs);
      GreyKv=SmartDashboard.getNumber("shooter Grey kV", GreyKv);
      GreyKa=SmartDashboard.getNumber("shooter Grey kA", GreyKa);
      GreyKp=SmartDashboard.getNumber("shooter Grey kP", GreyKp);
      Greyclpo=SmartDashboard.getNumber("shooterGrey clpo", Greyclpo);
      GreyFF=new SimpleMotorFeedforward(GreyKs,GreyKv,GreyKa);     
      shooterGrey.config_kP(0, GreyKp);
      shooterGrey.configClosedLoopPeakOutput(0, Greyclpo);
    }



    @Override
    public void periodic() {
      currentShooterSpeedL=shooterBlue.getSelectedSensorVelocity()*600/2048;
      currentShooterSpeedR=shooterGrey.getSelectedSensorVelocity()*600/2048;

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
      SmartDashboard.putNumber("shooterBlue Vel",currentShooterSpeedL);
      SmartDashboard.putNumber("shooterGrey Vel",currentShooterSpeedR);
      }
}
