// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.IntSupplier;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.RotateInPlace;
import frc.robot.commands.TurboToggle;
import frc.robot.Sensors.Camera;
import frc.robot.Utilities.DriverStationInterface;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.UpdatePID;
import frc.robot.commands.UpdateTable;
import frc.robot.commands.FollowCamera;
import frc.robot.commands.HeadingBumpCCW;
import frc.robot.commands.HeadingBumpCW;
import frc.robot.subsystems.SwerveDriveSystem;

import frc.robot.subsystems.Tables;
import frc.robot.subsystems.TrajectoryFollower;
import frc.robot.subsystems.CameraInterface;
import frc.robot.subsystems.Stick;
 
public class RobotContainer {
  Constants constants = new Constants();
  public final Stick stick =new Stick();
  public final Tables m_tables = new Tables();
  public final SwerveDriveSystem m_swervedriveSystem = new SwerveDriveSystem(m_tables);
  public final CameraInterface m_camInt = new CameraInterface(m_swervedriveSystem);
  public final TrajectoryFollower trajectoryFollower = new TrajectoryFollower(m_swervedriveSystem);
  public final DriverStationInterface driverInterface = new DriverStationInterface(m_swervedriveSystem);
  public final Camera camera = new Camera(); 
  final DriveSwerve m_driveswerve ;
  final UpdateTable m_updatetable ;
  final RotateInPlace m_RotateInPlace ;
  final FollowCamera m_followCamera ;
  IntSupplier povSelection = () -> stick.getPOV();
  Supplier<double[]> stickState = () -> stick.getStickState();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveswerve = new DriveSwerve(m_swervedriveSystem, stickState);
    m_updatetable = new UpdateTable(m_tables);
    m_RotateInPlace = new RotateInPlace(m_swervedriveSystem,povSelection);
    m_followCamera = new FollowCamera(m_camInt);
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
        new FollowCamera(m_camInt)); 
  }


  public Command getAutonomousCommand() {    
    return trajectoryFollower.getFollowTrajCommand();
  }


  } 
