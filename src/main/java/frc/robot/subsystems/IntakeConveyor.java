package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
//    PneumaticHub hub = new PneumaticHub();
    boolean pistonState=true;
    DoubleSolenoid solLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8,9);
    DoubleSolenoid solRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0,15);

    

    double intakeSpeed=0.50;
    
    boolean ballAtTop=irTopSensor.get();
    boolean ballAtBottom=irBottomSensor.get();
    boolean inAuto=false;
public IntakeConveyor(){
    intake.setStatusFramePeriod(1, 251);
    intake.setStatusFramePeriod(2, 241);
    intake.setStatusFramePeriod(4, 239);
    intake.setStatusFramePeriod(8, 233);
    intake.setStatusFramePeriod(10, 229);
    intake.setStatusFramePeriod(12, 227);
    intake.setStatusFramePeriod(13, 223);
    intake.setStatusFramePeriod(14, 211);
    intake.setStatusFramePeriod(21, 199);
    conveyorL.setStatusFramePeriod(1, 251);
    conveyorL.setStatusFramePeriod(2, 231);
    conveyorL.setStatusFramePeriod(4, 239);
    conveyorL.setStatusFramePeriod(8, 233);
    conveyorL.setStatusFramePeriod(10, 229);
    conveyorL.setStatusFramePeriod(12, 227);
    conveyorL.setStatusFramePeriod(13, 223);
    conveyorL.setStatusFramePeriod(14, 211);
    conveyorL.setStatusFramePeriod(21, 199); 
    conveyorU.setStatusFramePeriod(1, 251);
    conveyorU.setStatusFramePeriod(2, 231);
    conveyorU.setStatusFramePeriod(4, 239);
    conveyorU.setStatusFramePeriod(8, 233);
    conveyorU.setStatusFramePeriod(10, 229);
    conveyorU.setStatusFramePeriod(12, 227);
    conveyorU.setStatusFramePeriod(13, 223);
    conveyorU.setStatusFramePeriod(14, 211);
    conveyorU.setStatusFramePeriod(21, 199);

    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 251);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 241);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 239);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus,255);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,255);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 233);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 229);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,255);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 227);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 223);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 211);
    intake.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,255);

    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 251);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 241);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 239);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus,255);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,255);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 233);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 229);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,255);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 227);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 223);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 211);
    conveyorU.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,255);

    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 251);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 241);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 239);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus,255);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer,255);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 233);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 229);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,255);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 227);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 223);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 211);
    conveyorL.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,255);


    conveyorL.setStatusFramePeriod(1, 251);
    conveyorL.setStatusFramePeriod(2, 231);
    conveyorL.setStatusFramePeriod(4, 239);
    conveyorL.setStatusFramePeriod(8, 233);
    conveyorL.setStatusFramePeriod(10, 229);
    conveyorL.setStatusFramePeriod(12, 227);
    conveyorL.setStatusFramePeriod(13, 223);
    conveyorL.setStatusFramePeriod(14, 211);
    conveyorL.setStatusFramePeriod(21, 199); 
    conveyorU.setStatusFramePeriod(1, 251);
    conveyorU.setStatusFramePeriod(2, 231);
    conveyorU.setStatusFramePeriod(4, 239);
    conveyorU.setStatusFramePeriod(8, 233);
    conveyorU.setStatusFramePeriod(10, 229);
    conveyorU.setStatusFramePeriod(12, 227);
    conveyorU.setStatusFramePeriod(13, 223);
    conveyorU.setStatusFramePeriod(14, 211);
    conveyorU.setStatusFramePeriod(21, 199);


    solLeft.set(DoubleSolenoid.Value.kReverse);
    solRight.set(DoubleSolenoid.Value.kReverse);
    led3.set(false);
    intake.configFactoryDefault();
    intake.setNeutralMode(NeutralMode.Brake);
    intake.configOpenloopRamp(0.2);
    
/*    SupplyCurrentLimitConfiguration supplyconfig = 
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
    
    if (solLeft.get()==DoubleSolenoid.Value.kForward){
        solLeft.set(DoubleSolenoid.Value.kReverse);
        solRight.set(DoubleSolenoid.Value.kReverse);
    }
    else{
        solLeft.set(DoubleSolenoid.Value.kForward);
        solRight.set(DoubleSolenoid.Value.kForward);
    }
}


public void intakeUp(){
    if (solLeft.get()==DoubleSolenoid.Value.kForward){
        solLeft.set(DoubleSolenoid.Value.kReverse);
        solRight.set(DoubleSolenoid.Value.kReverse);
    }
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
    if (!inAuto) intakeUp();
  }

  SmartDashboard.putBoolean("lower IR snesor",ballAtBottom);
  SmartDashboard.putBoolean("top IR snesor", ballAtTop);
}


}
