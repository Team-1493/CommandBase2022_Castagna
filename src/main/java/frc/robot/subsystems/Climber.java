package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Climb.AutoZero;

public class Climber extends SubsystemBase {
  int leftID=14, rightID=15;
  TalonFX climbMotorL = new TalonFX(leftID);
  TalonFX climbMotorR = new TalonFX(rightID);
  
  int lowPos=2000;
  int medPos=2000;
  int highPos=2000;
  double climb_kP=.1;



public Climber(){
    climbMotorL.configFactoryDefault();
    climbMotorR.configFactoryDefault();
    climbMotorL.setNeutralMode(NeutralMode.Brake);
    climbMotorL.setNeutralMode(NeutralMode.Brake);
    climbMotorR.follow(climbMotorL);
 
    climbMotorL.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 25);
    climbMotorL.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 25);
    climbMotorL.setSelectedSensorPosition(0, 0, 25);
    climbMotorR.setSelectedSensorPosition(0, 0, 25);
 
    SmartDashboard.putNumber("climb_kP",climb_kP);
    
    climbMotorL.config_kP(0,climb_kP);
    climbMotorR.config_kP(0,climb_kP);


    climbMotorL.config_kD(0, 0.0);
    climbMotorL.config_kF(0, 0.0);
    climbMotorL.configClosedLoopPeakOutput(0, 1);
    climbMotorR.config_kD(0, 0.0);
    climbMotorR.config_kF(0, 0.0);
    climbMotorR.configClosedLoopPeakOutput(0, 1);
    
    

    climbMotorL.setInverted(InvertType.InvertMotorOutput);
    climbMotorL.setInverted(InvertType.FollowMaster);
    climbMotorR.setSensorPhase(false);

    
    climbMotorL.setSensorPhase(false);
    climbMotorR.setSensorPhase(false);

    climbMotorL.configReverseSoftLimitThreshold(0);
    climbMotorL.configReverseSoftLimitEnable(false, 25);

//    climbMotorL.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    climbMotorR.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    StatorCurrentLimitConfiguration statorconfig = 
        new StatorCurrentLimitConfiguration(true,25,30,0.5);        

    climbMotorL.set(ControlMode.PercentOutput, 0);
    climbMotorR.set(ControlMode.PercentOutput, 0);

    Command autozero = new AutoZero(this);
    //autozero.schedule();
}

    public void climbUp(){
        climbMotorL.set(ControlMode.Follower,15);
        climbMotorR.set(ControlMode.PercentOutput,0.5);
        SmartDashboard.putNumber("RLS", getRightLimitSwitch());
      if(getRightLimitSwitch()==1)
        climbMotorL.setSelectedSensorPosition(0, 0, 25);
        if(getRightLimitSwitch()==1)
        climbMotorR.setSelectedSensorPosition(0, 0, 25);

    }

    public void climbDown(){
        climbMotorL.set(ControlMode.Follower,15);        
        climbMotorR.set(ControlMode.PercentOutput,-0.5);
      if(getRightLimitSwitch()==1)
        climbMotorL.setSelectedSensorPosition(0, 0, 25);
        if(getRightLimitSwitch()==1)
        climbMotorR.setSelectedSensorPosition(0, 0, 25);

    }


    public void climbPosition(int position){
        climbMotorR.set(ControlMode.Position, position);
        climbMotorL.set(ControlMode.Follower,15);
    }


    public void stop(){
        climbMotorL.set(ControlMode.PercentOutput, 0);
        climbMotorR.set(ControlMode.Follower,leftID);
    }



    public double getLeftPosition(){
        return climbMotorL.getSelectedSensorPosition(0);
    }

    public double getRightPosition(){
        return climbMotorR.getSelectedSensorPosition(0);
    }   

    public int getLeftLimitSwitch(){
        return climbMotorL.getSensorCollection().isRevLimitSwitchClosed();
    }

    public int getRightLimitSwitch(){
        return climbMotorR.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public void setPositionToZero(){
        climbMotorL.setSelectedSensorPosition(0,0,25);
        climbMotorR.setSelectedSensorPosition(0,0,25);
    }

    public void setClimb_kP(){
        double DashboardClimb_kP = SmartDashboard.getNumber("climb_kP",climb_kP);
        if (DashboardClimb_kP != climb_kP){
            climbMotorL.config_kP(0,DashboardClimb_kP);
            climbMotorR.config_kP(0,DashboardClimb_kP);
            climb_kP = DashboardClimb_kP;
            
        }
    }

}
