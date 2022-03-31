// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.TurboToggle;
import frc.robot.Sensors.Camera;
//import frc.robot.Sensors.BallFollowCamera;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.UpdatePID;
import frc.robot.commands.UpdateTable;
import frc.robot.commands.Climb.ClimbManual;
import frc.robot.commands.Gyro.ReEnableGyro;
import frc.robot.commands.Gyro.ResetGyro;
import frc.robot.commands.IntakeShooter.IntakeBall;
import frc.robot.commands.IntakeShooter.ReverseIntake;
import frc.robot.commands.IntakeShooter.ShootBall;
import frc.robot.commands.IntakeShooter.ShootBallAuto;
import frc.robot.commands.LimelightFollowing.LimelightAlign;
import frc.robot.commands.LimelightFollowing.LimelightMove;
import frc.robot.commands.Rotate.HeadingBumpCCW;
import frc.robot.commands.Rotate.HeadingBumpCW;
import frc.robot.commands.Rotate.RotateInPlace;
import frc.robot.commands.FollowBall;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.subsystems.Tables;
import frc.robot.subsystems.AutoGenerator;
import frc.robot.subsystems.BallFollowInterface;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeConveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Stick;
 
public class RobotContainer {
public final Tables m_tables = new Tables();

// Joysticks
  public final Stick stick =new Stick(0);
  public final Stick operatorStick =new Stick(1);

// Driver Buttons
  public JoystickButton btnResetGyro = new JoystickButton(stick.getStick(),2);
  public JoystickButton btnUpdateConstants = new JoystickButton(stick.getStick(),3);

  public JoystickButton btnBumpCCW = new JoystickButton(stick.getStick(),5);
  public JoystickButton btnBumpCW = new JoystickButton(stick.getStick(),6);
  public JoystickButton btnFollowBall = new JoystickButton(stick.getStick(),9);
  public JoystickButton btnTurbo = new JoystickButton(stick.getStick(),6);


 // Operator Buttons 
  public JoystickButton btnShootBallHigh = new JoystickButton(operatorStick.getStick(),1);
  public JoystickButton btnClimbPositionDown = new JoystickButton(operatorStick.getStick(),2);
  public JoystickButton btnClimbPositionUp = new JoystickButton(operatorStick.getStick(),4);

  public JoystickButton btnLimelightAlign = new JoystickButton(operatorStick.getStick(),5);
  public JoystickButton btnShootBallLow = new JoystickButton(operatorStick.getStick(),6);
  public JoystickButton btnIntakeBall = new JoystickButton(operatorStick.getStick(),7);
  public JoystickButton btnShootBallManual = new JoystickButton(operatorStick.getStick(),8);

  public JoystickButton btnReverseIntake = new JoystickButton(operatorStick.getStick(),3);
  
  public JoystickButton btnClimbUpManual = new JoystickButton(operatorStick.getStick(),13);
  public JoystickButton btnClimbDownManual = new JoystickButton(operatorStick.getStick(),14);

  
  

  
  
 // Subsystems
  public final SwerveDriveSystem m_swervedriveSystem = new SwerveDriveSystem(m_tables);
  public final Shooter shooter = new Shooter();
  public final IntakeConveyor intake = new IntakeConveyor();
  public final Climber  m_climber  = new Climber();  
   public final BallFollowInterface m_ballFollower = new BallFollowInterface(m_swervedriveSystem);

  //  public final BallFollowCamera camera = new BallFollowCamera(); 

// Camera
public final Camera camera = new Camera(); 

 // Joystick Input Suppliers 
  Supplier<double[]> stickState = () -> stick.getStickState();
  IntSupplier povSelection = () -> stick.getPOV();

 // Commands 
  public final DriveSwerve m_driveswerve = new DriveSwerve(m_swervedriveSystem, stickState) ;
  public final UpdateTable m_updatetable = new UpdateTable(m_tables);
  public final RotateInPlace m_RotateInPlace = new RotateInPlace(m_swervedriveSystem,povSelection); ;
  public final FollowBall m_followBall = new FollowBall(m_ballFollower);
  //public final Command m_limelightAlign  = new LimelightAlign(m_swervedriveSystem,btnLimelightAlign);
  public final Command m_limelightMove  = new LimelightMove(m_swervedriveSystem,btnLimelightAlign, stickState, shooter);
  public final Command m_intakeBall  = new IntakeBall(intake, btnIntakeBall); 
  public final Command m_reverseIntake  = new ReverseIntake(intake, btnReverseIntake); 
  //  public final Command m_spimWheel  = new SpinWheel(shooter, btnSpinWheels);
public final Command m_shootBallAuto  = new ShootBallAuto(intake, shooter,1); 
  public final Command m_shootBallHigh  = new ShootBall(intake, shooter, btnShootBallHigh,1);
  public final Command m_shootBallLow  = new ShootBall(intake, shooter, btnShootBallLow,2);
  public final Command m_shootBallManual  = new ShootBall(intake, shooter, btnShootBallManual,3);

  public final Command m_climbUpManual  = new ClimbManual(m_climber,  btnClimbUpManual, -1);
  public final Command m_climbDownManual  = new ClimbManual(m_climber,  btnClimbDownManual, 1);  


  public final ReEnableGyro m_ReEnableGyro = new ReEnableGyro(m_swervedriveSystem) ;

  public final AutoGenerator autoGenerator = new AutoGenerator(m_swervedriveSystem, intake,shooter);

  public RobotContainer() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    m_swervedriveSystem.setDefaultCommand(m_driveswerve);
    m_tables.setDefaultCommand(m_updatetable);    
    configureButtonBindings();
  }
 
  public void configureButtonBindings() {
    new POVButton(stick.getStick(),0).whenPressed(m_RotateInPlace);
    new POVButton(stick.getStick(), 270).whenPressed(m_RotateInPlace);
    new POVButton(stick.getStick(), 180).whenPressed(m_RotateInPlace);
    new POVButton(stick.getStick(), 90).whenPressed(m_RotateInPlace);
    
    btnResetGyro.whenPressed(new ResetGyro(m_swervedriveSystem)); 
    btnBumpCCW.whenPressed(new HeadingBumpCCW(m_swervedriveSystem));
    btnBumpCW.whenPressed(new HeadingBumpCW(m_swervedriveSystem));
    btnTurbo.whenPressed(new TurboToggle(stick));

    //btnLimelightAlign.whenPressed(m_limelightAlign); 
    btnLimelightAlign.whenPressed(m_limelightMove); 
    btnIntakeBall.whenPressed(m_intakeBall); 
    btnReverseIntake.whenPressed(m_reverseIntake);
    btnShootBallHigh.whenPressed(m_shootBallHigh); 
    btnShootBallManual.whenPressed(m_shootBallManual); 
    btnShootBallLow.whenPressed(m_shootBallLow); 

    btnClimbUpManual.whileHeld(m_climbUpManual);
    btnClimbDownManual.whileHeld(m_climbDownManual);
    btnClimbPositionDown.whenPressed(new InstantCommand(()-> m_climber.climbPositionLower() ));
    btnClimbPositionUp.whenPressed(new InstantCommand(()-> m_climber.climbPositionHigher() ));


    btnUpdateConstants.whenPressed(new UpdatePID(m_swervedriveSystem, shooter));
   // btnResetEncoders.whenPressed(new ResetEncoders(m_swervedriveSystem));
  //  btnFollowBall.whileHeld(m_followBall); 
  }

  public SequentialCommandGroup getAutonomousCommand1() {    
    return autoGenerator.getAuto1();
  }

  public SequentialCommandGroup getAutonomousCommand2() {    
    return autoGenerator.getAuto2();
  }

  public SequentialCommandGroup getAutonomousCommand3() {    
    return autoGenerator.getAuto3();
  }

  public SequentialCommandGroup getAutonomousCommand4() {    
    return autoGenerator.getAuto4();
  }

  public SequentialCommandGroup getAutonomousCommand5() {    
    return autoGenerator.getAuto5();
  }
  public SequentialCommandGroup getAutonomousCommand6() {    
    return autoGenerator.getAuto6();
  }


  public void reEnableGyro(){
    m_ReEnableGyro.resetGryoAndRobotHeading();
  }

  public void setPIDslot(int slot){
    m_swervedriveSystem.setPIDSlot(slot);
  }

  public void updateConstants(){
    shooter.updateConstants();
    m_swervedriveSystem.updateConstants();
  }
  } 
