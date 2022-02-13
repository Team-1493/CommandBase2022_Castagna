// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.RotateInPlace;
import frc.robot.commands.TurboToggle;
import frc.robot.Sensors.Camera;
import frc.robot.Sensors.BallFollowCamera;
import frc.robot.Utilities.DriverStationInterface;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.UpdatePID;
import frc.robot.commands.UpdateTable;
import frc.robot.commands.LimelightFollowing.LimelightAlign;
import frc.robot.commands.LimelightFollowing.LimelightAutoTarget;
import frc.robot.commands.FollowBall;
import frc.robot.commands.HeadingBumpCCW;
import frc.robot.commands.HeadingBumpCW;
import frc.robot.subsystems.SwerveDriveSystem;

import frc.robot.subsystems.Tables;
import frc.robot.subsystems.TrajectoryFollower;
import frc.robot.subsystems.BallFollowInterface;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.Stick;
 
public class RobotContainer {
  Constants constants = new Constants();
//  PowerDistribution pdHub = new PowerDistribution(1, ModuleType.kRev);
  public final Stick stick =new Stick();
  public final Tables m_tables = new Tables();
  public final SwerveDriveSystem m_swervedriveSystem = new SwerveDriveSystem(m_tables);
  public final TrajectoryFollower trajectoryFollower = new TrajectoryFollower(m_swervedriveSystem);
  public final DriverStationInterface driverInterface = new DriverStationInterface(m_swervedriveSystem);
//  public final BallFollowCamera camera = new BallFollowCamera(); 
  public final LimelightInterface m_LimelightInterface = new LimelightInterface(m_swervedriveSystem);

public final BallFollowInterface m_ballFollower = new BallFollowInterface(m_swervedriveSystem);
Supplier<double[]> stickState = () -> stick.getStickState();
  final DriveSwerve m_driveswerve ;
  final UpdateTable m_updatetable ;
  final RotateInPlace m_RotateInPlace ;
   final FollowBall m_followBall ;
   final Command m_limelightAutoTarget  = new LimelightAutoTarget(m_swervedriveSystem,stickState);
  IntSupplier povSelection = () -> stick.getPOV();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  //  pdHub.setSwitchableChannel(true);
    m_driveswerve = new DriveSwerve(m_swervedriveSystem, stickState);
    m_updatetable = new UpdateTable(m_tables);
    m_RotateInPlace = new RotateInPlace(m_swervedriveSystem,povSelection);
    m_followBall = new FollowBall(m_ballFollower);
    m_swervedriveSystem.setDefaultCommand(m_driveswerve);
    m_tables.setDefaultCommand(m_updatetable);    
    configureButtonBindings();

  }
 
  public void configureButtonBindings() {
    new POVButton(stick.getStick(), Constants.pov_rotate0).whenPressed(m_RotateInPlace);
    new POVButton(stick.getStick(), Constants.pov_rotate90).whenPressed(m_RotateInPlace);
    new POVButton(stick.getStick(), Constants.pov_rotate180).whenPressed(m_RotateInPlace);
    new POVButton(stick.getStick(), Constants.pov_rotate270).whenPressed(m_RotateInPlace);

    new JoystickButton(stick.getStick(), Constants.btn_turbo[Constants.stickNum]).
    whenPressed(new TurboToggle(stick));

    new JoystickButton(stick.getStick(), Constants.btn_bumpCCW[Constants.stickNum]).
    whenPressed(new HeadingBumpCCW(m_swervedriveSystem));

    new JoystickButton(stick.getStick(), Constants.btn_bumpCW[Constants.stickNum]).whenPressed(
        new HeadingBumpCW(m_swervedriveSystem));

    new JoystickButton(stick.getStick(), Constants.btn_updatePID[Constants.stickNum]).whenPressed(
      new UpdatePID(m_swervedriveSystem));

    new JoystickButton(stick.getStick(), Constants.btn_resetencoder[Constants.stickNum]).whenPressed(
      new ResetEncoders(m_swervedriveSystem));

//    new JoystickButton(stick.getStick(), 10).whenPressed(
//        new CalibrateGyro(m_swervedriveSystem));

    new JoystickButton(stick.getStick(), Constants.btn_resetgyro[Constants.stickNum]).whenPressed(
        new ResetGyro(m_swervedriveSystem)); 

    new JoystickButton(stick.getStick(), Constants.btn_followcam[Constants.stickNum]).whileHeld(
        m_followBall); 

    new JoystickButton(stick.getStick(),14).whenPressed(m_limelightAutoTarget); 
  }


  public Command getAutonomousCommand() {    
    return trajectoryFollower.getFollowTrajCommand();
  }


  } 
