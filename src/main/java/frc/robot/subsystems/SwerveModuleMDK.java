// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 
import frc.robot.Utilities.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;




public class SwerveModuleMDK{
    public final TalonFX m_drive;
    private final TalonFX m_turn;
    private final CANCoder e_turn;
    SimpleMotorFeedforward feedforward_drive;
    double voltageComp=11;


    // Robot Dimensions for MK4 Swerve
    private  double  wheelDiamInches = 3.7435;//0.10033 meters 
    private double wheelCircumferenceMaters=wheelDiamInches*Math.PI*0.0254; //0.315195
    private double gearRatioDrive=8.1428; 
    private double MPSToRPM = 60.0*gearRatioDrive/wheelCircumferenceMaters;  // 1,550.0499
    private double MPSToNativeSpeed = MPSToRPM*2048.0/600.0;  // convert m/sec to Talon speed unit (counts/100ms)
    //scales the stick -1 to 1 input to meters/sec
    private double RadiansToNativePos=4096.0/(2*Math.PI);



    // Drive Motor Constants
    private double kP_drive=0.11;  //1.19 from characterization;
    private double kF_drive=0.052;   // 1023/20660
    private double kD_drive=0.0;   // 1023/20660
    private double kS_drive= 0.471;  // Volts  0.539 from characterization


   // Drive Motor Constants for auto
   private double kP_driveAuto=0.11;  //1.19 from characterization;
   private double kF_driveAuto=0.0452;   // 1023/20660
   private double kD_driveAuto=0.0;   // 1023/20660
   private double kS_driveAuto= 0.471;  // Volts  0.539 from characterization
  
//  Turn (Swerve) Motor Constants
    private double kP_turn=0.5; //0.5,   0.421 from characterization 
    private double kD_turn=0;  // 0 from characterization    
    private double kF_turn=0.0;   //  max turn RPM on ground = 485, 3310 units/100ms


// Motion Magic constants  (currently not used)
    private double SMMaxVelRadPerSec_turn=60;  
    private double SMMaxAccRadPerSec2_turn=60;
    private double SMMaxVel_turn= 0.1*4096*SMMaxVelRadPerSec_turn/(2*Math.PI);
    private double SMMaxAcc_turn=0.1*4096*SMMaxAccRadPerSec2_turn/(2*Math.PI);
    
    private double kMaxOutputDrive=1;
    private double kMaxOutputTurn=0.8;



    private final String driveVel=this.getClass().getName()+"_drive_vel";
    private final String drivePos=this.getClass().getName()+"_drive_pos";
    private final String turnVel=this.getClass().getName()+"_turn_vel";
    private final String turnPos=this.getClass().getName()+"_turn_pos";
    private final String CCAPos=this.getClass().getName()+"_turn_CCApos";
    
public SwerveModuleMDK(int driveID, int turnID, int cancoderID, double zeropos, 
                boolean invD, boolean invT, boolean invEncD, boolean invEncT){

                        
// set up the drive motor                                 
    m_drive=new TalonFX(driveID);
    m_drive.configFactoryDefault();
    m_drive.setInverted(invD);
    m_drive.configVoltageCompSaturation(voltageComp);
    m_drive.enableVoltageCompensation(false);
    m_drive.setNeutralMode(NeutralMode.Brake);
    m_drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0, 25);
    m_drive.configVelocityMeasurementWindow(8, 10);
    m_drive.setStatusFramePeriod(21, 20);
    m_drive.config_kP(0, kP_drive);
    m_drive.config_kF(0, kF_drive);
    m_drive.config_kD(0, kD_drive);
    m_drive.config_kP(1, kP_driveAuto);
    m_drive.config_kF(1, kF_driveAuto);
    m_drive.config_kD(1, kD_driveAuto);


    m_drive.configClosedLoopPeakOutput(0,kMaxOutputDrive);
                    


// set up the turn encoder
    e_turn=new CANCoder(cancoderID);
    e_turn.configSensorDirection(false);                    
    e_turn.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    e_turn.setPosition(e_turn.getAbsolutePosition()-zeropos);

    // set up the turn motor    
    m_turn=new TalonFX(turnID);
    m_turn.configFactoryDefault();
    m_turn.setInverted(invD);
    m_turn.setNeutralMode(NeutralMode.Brake);

    m_turn.configRemoteFeedbackFilter(e_turn, 0, 25);
    m_turn.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0,0,25);       
//    m_turn.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);                
    m_turn.configForwardSoftLimitEnable(false);
    m_turn.configReverseSoftLimitEnable(false);
    m_turn.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turn.configPeakOutputForward(1);
    m_turn.configPeakOutputReverse(-1);
    m_turn.configClosedLoopPeakOutput(0, 0.8);
    m_turn.configSelectedFeedbackCoefficient(1);
    m_turn.config_kP(0,kP_turn);
    m_turn.config_kD(0,kD_turn);
    m_turn.config_kF(0,kF_turn);
    
	m_turn.configMotionCruiseVelocity(SMMaxVel_turn, 25);
	m_turn.configMotionAcceleration(SMMaxAcc_turn, 25);


    SmartDashboard.putNumber("kD_Drive",kD_drive);
    SmartDashboard.putNumber("kP_Drive",kP_drive);
    SmartDashboard.putNumber("kF_Drive",kF_drive);
    SmartDashboard.putNumber("kS_Drive",kS_drive);

    SmartDashboard.putNumber("kD_DriveAuto",kD_driveAuto);
    SmartDashboard.putNumber("kP_DriveAuto",kP_driveAuto);
    SmartDashboard.putNumber("kF_DriveAuto",kF_driveAuto);
    SmartDashboard.putNumber("kS_DriveAuto",kS_driveAuto);

    

    SmartDashboard.putNumber("kP_Turn",kP_turn);
    SmartDashboard.putNumber("kD_Turn",kD_turn);
    SmartDashboard.putNumber("kF_Turn",kF_turn);
    SmartDashboard.putNumber("SMMaxVelRadPerSec_turn",SMMaxVelRadPerSec_turn);
    SmartDashboard.putNumber("SMMaxAccRadPerSec2_turn",SMMaxAccRadPerSec2_turn);
    SmartDashboard.putNumber("MaxOutput Turn",kMaxOutputTurn);

}


// get the module state for odometry calculations: velocity in mps and module angle in radians
public SwerveModuleState getState() {
    return new SwerveModuleState(
        velNativeToRPM_talon(m_drive.getSelectedSensorVelocity())/MPSToRPM, 
        new Rotation2d(getTurnPosition_Rad()));
  }


public void setMotors(double speed,double turnAngle) {
    m_drive.set(ControlMode.Velocity, speed*MPSToNativeSpeed+kS_drive*Math.signum(speed));
//m_drive.set(ControlMode.Velocity,  speed*MPSToNativeSpeed, DemandType.ArbitraryFeedForward, kS_drive);
    m_turn.set(ControlMode.Position,turnAngle*RadiansToNativePos);
}


public void setMotorsAllStop() {
    m_drive.set(ControlMode.PercentOutput,0);
    m_turn.set(ControlMode.PercentOutput,0);
}

  public void setMotorsFF(double speed,double turnAngle) {
    double driveFFUnits=feedforward_drive.calculate(speed)/voltageComp;
//    m_drive.set(ControlMode.Velocity, speed*MPSToNativeSpeed,DemandType.ArbitraryFeedForward,driveFFUnits);
    m_turn.set(TalonFXControlMode.MotionMagic,turnAngle*RadiansToNativePos);
  }

  // write encoder values to dashboard, units in RPM and deg
  public void writeEncoders() {
    SmartDashboard.putNumber(driveVel, getDriveVelocity());
    SmartDashboard.putNumber(drivePos, getDrivePosition());
    SmartDashboard.putNumber(turnVel, getTurnVelocity());
    SmartDashboard.putNumber(turnPos, getTurnPosition_Deg());
    SmartDashboard.putNumber(CCAPos, e_turn.getAbsolutePosition());    
  }

  public void resetEncoders() {
      m_drive.setSelectedSensorPosition(0);
     }

// get the turn encoder position, measured in rotations
     public double getTurnPosition_Rot() {

        return ( e_turn.getPosition()/360);
       }

// get the turn encoder position, measured in rotations
public double getTurnPosition_Deg() {
    return (e_turn.getPosition());
   }       

   // get the turn encoder position, measured in radians
public double getTurnPosition_Rad() { 
    return ( Util.toRadians(e_turn.getPosition()));
   }       
   
       // get the turn encoder absolute position, measured in degrees
     public double getTurnAbsPosition() {
        return e_turn.getAbsolutePosition();
       }

// get the turn velocity velocity, measured in rpm
public double getTurnVelocity() {
    return (m_turn.getSelectedSensorVelocity()*600/4096);
   }       

// get the turn output, measured in rotations
public double getTurnMotorCLT() {
    return m_turn.getClosedLoopTarget();
   }       

   // get the turn output, measured in rotations
public double getTurnMotorCLE() {
    return m_turn.getClosedLoopError();
   }       


// get the drive encoder position, measured in rotations
     public double getDrivePosition() {
        return (m_drive.getSelectedSensorPosition()/2048.);
       }

// get the drive encoder velocity, measured in rpm
     public double getDriveVelocity() {
        return  velNativeToRPM_talon(m_drive.getSelectedSensorVelocity());
       }

// get voltage
    public double getVoltage() {
        return  m_drive.getMotorOutputVoltage();
    }
    
// get the drive encoder velocity, measured in rpm
public double getDriveErrorRPM() {
    return  velNativeToRPM_talon(m_drive.getClosedLoopError());
   }

  


    private double velNativeToRPM_talon(double vel_NativeUnits){
        return  vel_NativeUnits*600/2048;
    }

    public double MPStoRPM(double mps){
        return mps*MPSToRPM;
    }

    public void updateConstants(){
        kP_drive= SmartDashboard.getNumber("kP_Drive",kP_drive);
        kF_drive= SmartDashboard.getNumber("kF_Drive",kF_drive);
        kD_drive= SmartDashboard.getNumber("kD_Drive",kD_drive);
        kS_drive= SmartDashboard.getNumber("kS_Drive",kS_drive);

        kP_driveAuto= SmartDashboard.getNumber("kP_DriveAuto",kP_driveAuto);
        kF_driveAuto= SmartDashboard.getNumber("kF_DriveAuto",kF_driveAuto);
        kD_driveAuto= SmartDashboard.getNumber("kD_DriveAuto",kD_driveAuto);
        kS_driveAuto= SmartDashboard.getNumber("kS_DriveAuto",kS_driveAuto);

//        kP_turn= SmartDashboard.getNumber("kP_Turn",kP_turn);
//        kD_turn= SmartDashboard.getNumber("kD_Turn",kD_turn);
//        kF_turn= SmartDashboard.getNumber("kF_Turn",kF_turn);
//        kMaxOutputTurn=SmartDashboard.getNumber("MaxOutput Turn",kMaxOutputTurn);

 //       SMMaxVelRadPerSec_turn= SmartDashboard.getNumber("SMMaxVelRadPerSec_turn",SMMaxVelRadPerSec_turn);
 //       SMMaxAccRadPerSec2_turn= SmartDashboard.getNumber("SMMaxAccRadPerSec2_turn",SMMaxAccRadPerSec2_turn);
 //       SMMaxVel_turn= 0.1*4096*SMMaxVelRadPerSec_turn/(2*Math.PI);
 //       SMMaxAcc_turn=0.1*4096*SMMaxAccRadPerSec2_turn/(2*Math.PI);
        m_drive.config_kF(0,kF_drive);
        m_drive.config_kP(0,kP_drive);
        m_drive.config_kD(0,kD_drive);
        m_drive.config_kF(1,kF_driveAuto);
        m_drive.config_kP(1,kP_driveAuto);
        m_drive.config_kD(1,kD_driveAuto);


//        m_turn.config_kP(0, kP_turn)
//        m_turn.config_kD(0,kD_turn);
//        m_turn.config_kF(0,kF_turn);
 //       m_turn.configClosedLoopPeakOutput(0, kMaxOutputTurn);                        
    }

    public void setPIDslot(int slot){
        m_drive.selectProfileSlot(slot, 0);
    }

}