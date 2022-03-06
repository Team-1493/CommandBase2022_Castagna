package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  int leftID=14, rightID=15;
  TalonFX climbMotorL = new TalonFX(leftID);
  TalonFX climbMotorR = new TalonFX(rightID);
  
    public int position=0;
    double climb_kP=.1;
    boolean zeroedLeft = false;
    boolean zeroedRight = false;
    int pos1=0,pos2 = 300000, pos3=507569,pos4=1042522;


public Climber(){
    climbMotorL.configFactoryDefault();
    climbMotorR.configFactoryDefault();
    climbMotorL.setNeutralMode(NeutralMode.Brake);
    climbMotorL.setNeutralMode(NeutralMode.Brake);
//    climbMotorR.follow(climbMotorL);
 
    climbMotorL.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 25);
    climbMotorR.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 25);
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
    
    
    climbMotorL.setSensorPhase(false);
    climbMotorR.setSensorPhase(false);

    climbMotorL.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    climbMotorR.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);


    climbMotorL.set(ControlMode.PercentOutput, 0);
    climbMotorR.set(ControlMode.PercentOutput, 0);

    if(getRightLimitSwitch()==1) zeroedRight=true;
    if(getRightLimitSwitch()==1) zeroedLeft=true;


}

/// This is really down
    public void climbUp(){
        if(getLeftLimitSwitch()==1){
            climbMotorL.set(ControlMode.PercentOutput,0);
            climbMotorL.setSelectedSensorPosition(0, 0, 25);
            zeroedLeft=true;
        }
        else climbMotorL.set(ControlMode.PercentOutput,-0.5);

        if(getRightLimitSwitch()==1){        
            climbMotorR.set(ControlMode.PercentOutput,0);
            climbMotorR.setSelectedSensorPosition(0, 0, 25);
            zeroedRight=true;
        }
        else climbMotorR.set(ControlMode.PercentOutput,-0.5);
    }

/// This is really up    
    public void climbDown(){    
        if(zeroedLeft && climbMotorL.getSelectedSensorPosition(0)>1100000)
            climbMotorL.set(ControlMode.PercentOutput,0.0);
        else 
            climbMotorL.set(ControlMode.PercentOutput,0.5);

        if(zeroedRight && climbMotorR.getSelectedSensorPosition(0)>1100000)
            climbMotorR.set(ControlMode.PercentOutput,0.0);
        else
            climbMotorR.set(ControlMode.PercentOutput,0.5);
    }


    public void climbDownRight(){
        climbMotorR.set(ControlMode.PercentOutput,0.5);
        if(getRightLimitSwitch()==1){
            climbMotorR.setSelectedSensorPosition(0, 0, 25);
            zeroedRight=true;    
        }

    }

    public void climbDownLeft(){
        climbMotorL.set(ControlMode.PercentOutput,0.5);        
        if(getRightLimitSwitch()==1){
            climbMotorL.setSelectedSensorPosition(0, 0, 25);
            zeroedLeft=true;
        }    
    }


    public void climbUpRight(){
        climbMotorR.set(ControlMode.PercentOutput,-0.5);
    }

    public void climbUpLeft(){
        climbMotorL.set(ControlMode.PercentOutput,-0.5);        
    }


public void climbPositionHigher(){
    int pos=0;

    if (position<4) position++;
    if (position==3) position=4;

    if (position==1) pos=pos1;
    if(position==2) pos=pos2;
    if(position==3) pos=pos3;
    if(position==4) pos=pos4;

    if(zeroedLeft && zeroedRight){
        climbMotorR.set(ControlMode.Position, pos);
        climbMotorL.set(ControlMode.Follower,15);
    }
}
    

public void climbPositionLower(){
    int pos=0;

    if (position>1) position--;

    if (position==1) pos=pos1;
    if(position==2) pos=pos2;
    if(position==3) pos=pos3;
    if(position==4) pos=pos4;

    if(zeroedLeft && zeroedRight){
        climbMotorR.set(ControlMode.Position, pos);
        climbMotorL.set(ControlMode.Follower,15);
    }
}

    public void climbPosition(int position){
        if(zeroedLeft && zeroedRight){
        climbMotorR.set(ControlMode.Position, position);
        climbMotorL.set(ControlMode.Follower,15);
        }
    }


    public void stop(){
        climbMotorL.set(ControlMode.PercentOutput, 0);
        climbMotorR.set(ControlMode.PercentOutput, 0);
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
        return climbMotorR.getSensorCollection().isRevLimitSwitchClosed();
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
