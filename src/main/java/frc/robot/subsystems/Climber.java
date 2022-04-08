package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  int leftID=14, rightID=15;
  TalonFX climbMotorL = new TalonFX(leftID);
  TalonFX climbMotorR = new TalonFX(rightID);
    double currentLimit=35;
    public int position=0;
    double climb_kP=.05;
    boolean zeroedLeft = false;
    boolean zeroedRight = false;
    int pos1L=0,pos1R=0;
    int pos2L = 100000,pos2R = 100000;
    int pos3L=169189,pos3R=169189;
    int pos4L=390000,pos4R=340000;


public Climber(){
    climbMotorL.configFactoryDefault();
    climbMotorR.configFactoryDefault();

    climbMotorL.setStatusFramePeriod(4, 251);
    climbMotorL.setStatusFramePeriod(8, 241);
    climbMotorL.setStatusFramePeriod(10, 239);
    climbMotorL.setStatusFramePeriod(12, 233);
    climbMotorL.setStatusFramePeriod(14, 229);

    climbMotorR.setStatusFramePeriod(4, 251);
    climbMotorR.setStatusFramePeriod(8, 241);
    climbMotorR.setStatusFramePeriod(10, 239);
    climbMotorR.setStatusFramePeriod(12, 233);
    climbMotorR.setStatusFramePeriod(14, 229);

    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 251);
    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 239);
    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus,255);
    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,255);
    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 233);
    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 229);
    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,255);
    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 227);
    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 211);
    climbMotorL.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,255);

    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 251);
    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 239);
    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus,255);
    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,255);
    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 233);
    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 229);
    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,255);
    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 227);
    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 211);
    climbMotorR.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,255);



//    climbMotorL.setInverted(InvertType.InvertMotorOutput);
    climbMotorR.setInverted(InvertType.InvertMotorOutput);

    climbMotorL.setNeutralMode(NeutralMode.Brake);
    climbMotorR.setNeutralMode(NeutralMode.Brake);
//    climbMotorR.follow(climbMotorL);
 
//    climbMotorL.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 25);
//    climbMotorR.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 25);
    climbMotorL.setSelectedSensorPosition(0, 0, 25);
    climbMotorR.setSelectedSensorPosition(0, 0, 25);

    climbMotorL.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true,currentLimit,currentLimit,0.1) );
    climbMotorR.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true,currentLimit,currentLimit,0.1) );

    climbMotorL.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true,currentLimit,currentLimit,0.1) );
    climbMotorR.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true,currentLimit,currentLimit,0.1) );

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
    climbMotorR.setSensorPhase(true);

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
        else climbMotorL.set(ControlMode.PercentOutput,-0.4);

        if(getRightLimitSwitch()==1){        
            climbMotorR.set(ControlMode.PercentOutput,0);
            climbMotorR.setSelectedSensorPosition(0, 0, 25);
            zeroedRight=true;
        }
        else climbMotorR.set(ControlMode.PercentOutput,-0.4);
    }

/// This is really up    
    public void climbDown(){    
        if(zeroedLeft && climbMotorL.getSelectedSensorPosition(0)>1100000)
            climbMotorL.set(ControlMode.PercentOutput,0.0);
        else 
            climbMotorL.set(ControlMode.PercentOutput,0.4);

        if(zeroedRight && climbMotorR.getSelectedSensorPosition(0)>1100000)
            climbMotorR.set(ControlMode.PercentOutput,0.0);
        else
            climbMotorR.set(ControlMode.PercentOutput,0.4);
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
    int posL=0,posR=0;

    if (position<4) position++;
    if (position==3) position=4;

    if (position==1) {posL=pos1L;posR=pos1R;}
    if(position==2) {posL=pos2L;posR=pos2R;}
    if(position==3) {posL=pos3L;posR=pos3R;}
    if(position==4) {posL=pos4L;posR=pos4R;}

    if(zeroedLeft && zeroedRight){
        climbMotorR.set(ControlMode.Position, posR);
        climbMotorL.set(ControlMode.Position, posL);
    }
}
    

public void climbPositionLower(){
    int posL=0,posR=0;

    if (position>1) position--;

    if (position==1) {posL=pos1L;posR=pos1R;}
    if(position==2) {posL=pos2L;posR=pos2R;}
    if(position==3) {posL=pos3L;posR=pos3R;}
    if(position==4) {posL=pos4L;posR=pos4R;}

    if(zeroedLeft && zeroedRight){
        climbMotorR.set(ControlMode.Position, posR);
        climbMotorL.set(ControlMode.Position, posL);
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
