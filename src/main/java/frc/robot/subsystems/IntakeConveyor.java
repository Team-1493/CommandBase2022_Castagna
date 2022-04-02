package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class IntakeConveyor extends SubsystemBase{
    TalonFX intake = new TalonFX(9);
    TalonFX conveyorU = new TalonFX(11);
    TalonFX conveyorL = new TalonFX(10);
    DigitalInput irTopSensor=new DigitalInput(7);
    DigitalInput irBottomSensor=new DigitalInput(8);
    DigitalOutput led1 = new DigitalOutput(0);
    DigitalOutput led2 = new DigitalOutput(1);
    DigitalOutput led3 = new DigitalOutput(2);
    PneumaticHub hub = new PneumaticHub();
    DoubleSolenoid solLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1,2);
    DoubleSolenoid solRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3,4);

    

    double intakeSpeed=0.40;
    
    boolean ballAtTop=irTopSensor.get();
    boolean ballAtBottom=irBottomSensor.get();
    
public IntakeConveyor(){
    solLeft.set(DoubleSolenoid.Value.kForward);
    solRight.set(DoubleSolenoid.Value.kForward);
    led3.set(false);
    intake.configFactoryDefault();
    intake.setNeutralMode(NeutralMode.Brake);
    intake.configOpenloopRamp(0.2);
/*    
    SupplyCurrentLimitConfiguration supplyconfig = 
        new SupplyCurrentLimitConfiguration(true,25,30,0.5);
    StatorCurrentLimitConfiguration statorconfig = 
        new StatorCurrentLimitConfiguration(true,25,30,0.5);        
    intake.configStatorCurrentLimit(statorconfig);
    intake.configSupplyCurrentLimit(supplyconfig);
 */   
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
    conveyorU.set(ControlMode.PercentOutput,0);
}

public void startIntakeLowerConveyor(){
    intake.set(ControlMode.PercentOutput,-intakeSpeed);
    conveyorL.set(ControlMode.PercentOutput,-0.25); 
}

public void stopIntakeLowerConveyor(){
    intake.set(ControlMode.PercentOutput,0);
    conveyorL.set(ControlMode.PercentOutput,0); 
}


public void startIntake(){
    intake.set(ControlMode.PercentOutput,-intakeSpeed);
}

public void reverseIntake(){
    intake.set(ControlMode.PercentOutput,intakeSpeed);
}


public void reverseIntakeAndConveyor(){
    intake.set(ControlMode.PercentOutput,intakeSpeed);
    conveyorL.set(ControlMode.PercentOutput,0.25); 
    conveyorU.set(ControlMode.PercentOutput,-0.25);
}



public void startLowerConveyor(){
    conveyorL.set(ControlMode.PercentOutput,-0.25); 
}


public void stopIntake(){
    intake.set(ControlMode.PercentOutput,0);
}

public void stopLowerConveyor(){
    conveyorL.set(ControlMode.PercentOutput,0); 
}


public boolean ballAtTop(){
    return !irTopSensor.get();
}

public boolean ballAtBottom(){
    return !irBottomSensor.get();
}

public void toggleIntake(){
    solLeft.toggle();
}

@Override
public void periodic() {
    ballAtTop=ballAtTop();
    ballAtBottom=ballAtBottom();
  if(!ballAtTop && !ballAtBottom){
    led1.set(false);led2.set(false);
  }
  else if (ballAtTop ^ ballAtBottom){
    led1.set(true);led2.set(false);
  }
  else {
    led1.set(true);led2.set(true);
  }
  SmartDashboard.putBoolean("lower IR snesor",ballAtBottom);
  SmartDashboard.putBoolean("top IR snesor", ballAtTop);
}


}
