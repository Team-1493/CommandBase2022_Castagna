package frc.robot.subsystems;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class IntakeConveyor extends SubsystemBase{
    TalonFX intake = new TalonFX(9);
    TalonFX conveyorU = new TalonFX(11);
    TalonFX conveyorL = new TalonFX(10);
    DigitalInput irTopSensor=new DigitalInput(9);
    DigitalInput irBottomSensor=new DigitalInput(7);

    double intakeSpeed=0.25;
    
    boolean ballAtTop=irTopSensor.get();
    boolean ballAtBottom=irBottomSensor.get();
    
public IntakeConveyor(){
    intake.configFactoryDefault();
    intake.setNeutralMode(NeutralMode.Brake);
    intake.configOpenloopRamp(0.2);
    SupplyCurrentLimitConfiguration supplyconfig = 
        new SupplyCurrentLimitConfiguration(true,25,30,0.5);
    StatorCurrentLimitConfiguration statorconfig = 
        new StatorCurrentLimitConfiguration(true,25,30,0.5);        

    conveyorU.configFactoryDefault();
    conveyorU.setNeutralMode(NeutralMode.Brake);
    conveyorU.configOpenloopRamp(0.2);

    conveyorL.configFactoryDefault();
    conveyorL.setNeutralMode(NeutralMode.Brake);
    conveyorL.configOpenloopRamp(0.2);  
  

}



public void startUpperConveyor(){
    conveyorU.set(ControlMode.PercentOutput,0.25);
}

public void stopUpperConveyor(){
    conveyorU.set(ControlMode.PercentOutput,0.25);
}

public void startIntakeLowerConveyor(){
    intake.set(ControlMode.PercentOutput,-intakeSpeed);
    conveyorL.set(ControlMode.PercentOutput,-0.25); 
}

public void stopIntakeLowerConveyor(){
    intake.set(ControlMode.PercentOutput,0);
    conveyorL.set(ControlMode.PercentOutput,0); 
}

public boolean ballAtTop(){
    return irTopSensor.get();
}

public boolean ballAtBottom(){
    return !irBottomSensor.get();
}


@Override
public void periodic() {
  SmartDashboard.putBoolean("lower IR snesor", ballAtBottom());
}


}
