package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    int leftID=14, rightID=15;
  TalonFX climbMotorL = new TalonFX(leftID);
  TalonFX climbMotorR = new TalonFX(rightID);
  

  int lowPos=2000;
  int medPos=2000;
  int highPos=2000;




public Climber(){
    climbMotorL.configFactoryDefault();
    climbMotorR.configFactoryDefault();
    climbMotorL.setNeutralMode(NeutralMode.Brake);
    climbMotorL.setNeutralMode(NeutralMode.Brake);
    climbMotorR.follow(climbMotorL);
 
    climbMotorL.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 25);
    climbMotorR.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 25);
 
    climbMotorL.set(ControlMode.PercentOutput, 0);
    climbMotorR.set(ControlMode.PercentOutput, 0);
}

    public void climbUp(){
        climbMotorL.set(ControlMode.PercentOutput, -0.5);
        climbMotorR.set(ControlMode.Follower,leftID);
    }

    public void climbDown(){
        climbMotorL.set(ControlMode.PercentOutput, 0.5);
        climbMotorR.set(ControlMode.Follower,leftID);
    }

    public void stop(){
        climbMotorL.set(ControlMode.PercentOutput, 0);
        climbMotorR.set(ControlMode.Follower,leftID);
    }

    public double climbLeftPosition(){
        return climbMotorL.getSelectedSensorPosition(0);
    }

    public double climbRightPosition(){
        return climbMotorR.getSelectedSensorPosition(0);
    }


}
