// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSystem;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveSwerve extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDriveSystem m_swervedrive;
  private final Supplier<double[]> m_stickState;



  public DriveSwerve(SwerveDriveSystem swervedrive, Supplier<double[]> stickState) {
 
    m_swervedrive = swervedrive;
    m_stickState=stickState;

    addRequirements(swervedrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      m_swervedrive.setMotors(m_stickState.get());      
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // m_swervedrive.setMotors(0.,0.,0.,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
