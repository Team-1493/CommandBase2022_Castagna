// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.TurboToggle;
import frc.robot.Sensors.Camera;
import frc.robot.Sensors.BallFollowCamera;
import frc.robot.Utilities.DriverStationInterface;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.UpdatePID;
import frc.robot.commands.UpdateTable;
import frc.robot.commands.Climb.ClimbManual;
import frc.robot.commands.Climb.ClimbPosition;
import frc.robot.commands.Gyro.ReEnableGyro;
import frc.robot.commands.Gyro.ResetGyro;
import frc.robot.commands.IntakeShooter.IntakeBall;
import frc.robot.commands.IntakeShooter.ShootBallHigh;
import frc.robot.commands.LimelightFollowing.LimelightAlign;
import frc.robot.commands.LimelightFollowing.LimelightAutoTarget;
import frc.robot.commands.Rotate.HeadingBumpCCW;
import frc.robot.commands.Rotate.HeadingBumpCW;
import frc.robot.commands.Rotate.RotateInPlace;
import frc.robot.commands.FollowBall;
import frc.robot.subsystems.SwerveDriveSystem;

import frc.robot.subsystems.Tables;
import frc.robot.subsystems.TrajectoryFollower;
import frc.robot.subsystems.BallFollowInterface;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeConveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Stick;
 
public class RobotContainer {
  Constants constants = new Constants();
public final Tables m_tables = new Tables();

// Joysticks
  public final Stick stick =new Stick(0);
  public final Stick operatorStick =new Stick(1);

// Driver Buttons
  public JoystickButton btnResetGyro = new JoystickButton(stick.getStick(),2);
  public JoystickButton btnBumpCCW = new JoystickButton(stick.getStick(),5);
  public JoystickButton btnBumpCW = new JoystickButton(stick.getStick(),6);
  public JoystickButton btnTurbo = new JoystickButton(stick.getStick(),10);
  public JoystickButton btnLimelightTarget = new JoystickButton(stick.getStick(),14);


 // Operator Buttons 
  public JoystickButton btnIntakeBall = new JoystickButton(operatorStick.getStick(),5);
  public JoystickButton btnShootBallLow = new JoystickButton(operatorStick.getStick(),6);
  public JoystickButton btnShootBallHigh = new JoystickButton(operatorStick.getStick(),7);
  public JoystickButton btnShootBallHighManual = new JoystickButton(operatorStick.getStick(),8);
  public JoystickButton btnClimbUpManual = new JoystickButton(operatorStick.getStick(),13);
  public JoystickButton btnClimbDownManual = new JoystickButton(operatorStick.getStick(),14);
  public JoystickButton btnClimbPosition1 = new JoystickButton(operatorStick.getStick(),2);
  public JoystickButton btnClimbPosition2 = new JoystickButton(operatorStick.getStick(),3);
  public JoystickButton btnClimbPosition3 = new JoystickButton(operatorStick.getStick(),4);
 
  


  
 // Subsystems
  public final SwerveDriveSystem m_swervedriveSystem = new SwerveDriveSystem(m_tables);
  public final Shooter shooter = new Shooter();
  public final IntakeConveyor intake = new IntakeConveyor();
  public final Climber  m_climber  = new Climber();  
  public final TrajectoryFollower trajectoryFollower = new TrajectoryFollower(m_swervedriveSystem);
  public final DriverStationInterface driverInterface = new DriverStationInterface(m_swervedriveSystem);
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
  public final Command m_limelightAutoTarget  = new LimelightAutoTarget(m_swervedriveSystem,stickState);
  public final Command m_intakeBall  = new IntakeBall(intake, btnIntakeBall);
  public final Command m_shootBallHigh  = new ShootBallHigh(intake, shooter, btnShootBallHigh);
  public final Command m_shootBallLow  = new ShootBallHigh(intake, shooter, btnShootBallLow);
  public final Command m_shootBallHighManual  = new ShootBallHigh(intake, shooter, btnShootBallHighManual);
  public final Command m_climbUpManual  = new ClimbManual(m_climber,  btnClimbUpManual, 1);
  public final Command m_climbDownManual  = new ClimbManual(m_climber,  btnClimbUpManual, -1);
  public final Command m_climbPosition1  = new ClimbPosition(m_climber,  1);
  public final Command m_climbPosition2  = new ClimbPosition(m_climber,  2);
  public final Command m_climbPosition3  = new ClimbPosition(m_climber,  3);
 
  ;
  
  public final ReEnableGyro m_ReEnableGyro = new ReEnableGyro(m_swervedriveSystem) ;


  public RobotContainer() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
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

    btnLimelightTarget.whenPressed(m_limelightAutoTarget); 
    btnIntakeBall.whenPressed(m_intakeBall); 
    btnShootBallHigh.whenPressed(m_shootBallHigh); 
    btnShootBallHighManual.whenPressed(m_shootBallHighManual); 
    btnShootBallLow.whenPressed(m_shootBallLow); 
    btnClimbDownManual.whileHeld(m_climbDownManual);
    btnClimbUpManual.whileHeld(m_climbUpManual);
    btnClimbUpManual.whileHeld(m_climbUpManual);
    btnClimbPosition1.whileHeld(m_climbPosition1);
    btnClimbPosition2.whileHeld(m_climbPosition2);
    btnClimbPosition3.whileHeld(m_climbPosition3);



    new JoystickButton(stick.getStick(), Constants.btn_updatePID[Constants.stickNum]).whenPressed(
      new UpdatePID(m_swervedriveSystem));

    new JoystickButton(stick.getStick(), Constants.btn_resetencoder[Constants.stickNum]).whenPressed(
      new ResetEncoders(m_swervedriveSystem));

    new JoystickButton(stick.getStick(), Constants.btn_followcam[Constants.stickNum]).whileHeld(
        m_followBall); 
  }


  public Command getAutonomousCommand() {    
    return trajectoryFollower.getFollowTrajCommand();
  }

  public void reEnableGyro(){
    m_ReEnableGyro.resetGryoAndRobotHeading();
  }


  } 
