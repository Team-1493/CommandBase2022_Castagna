// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Tables;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class UpdateTable extends CommandBase {
    private final Tables m_tables;  



  public UpdateTable(Tables tables) {
    m_tables=tables;
    addRequirements(m_tables);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_tables.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}

