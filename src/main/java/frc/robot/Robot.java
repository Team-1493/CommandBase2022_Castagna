// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private SequentialCommandGroup m_autonomousCommand1,m_autonomousCommand2,m_autonomousCommand3;
  private SequentialCommandGroup m_autonomousCommand4,m_autonomousCommand5,m_autonomousCommand6,m_autonomousCommand7;
  private SequentialCommandGroup m_autonomousCommand;
  private String m_autoSelected;
  
  private RobotContainer m_robotContainer;
  Timer timer = new Timer();
  boolean shootflag=false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    LiveWindow.disableAllTelemetry();
    m_robotContainer = new RobotContainer();
    SmartDashboard.putData(CommandScheduler.getInstance());
    m_robotContainer.reEnableGyro();
    m_autonomousCommand1 = m_robotContainer.getAutonomousCommand1();
    m_autonomousCommand2 = m_robotContainer.getAutonomousCommand2();
    m_autonomousCommand3 = m_robotContainer.getAutonomousCommand3();
    m_autonomousCommand4 = m_robotContainer.getAutonomousCommand4();
    m_autonomousCommand5 = m_robotContainer.getAutonomousCommand5();
    m_autonomousCommand6 = m_robotContainer.getAutonomousCommand6();
    m_autonomousCommand7 = m_robotContainer.getAutonomousCommand7();
  }


  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setAutoFlag(true);
    m_autonomousCommand6 = m_robotContainer.getAutonomousCommand6();
    m_robotContainer.setPIDslot(1);  // use the auto PID gains for teleop
    m_autoSelected=SmartDashboard.getString("Auto Selector", "5");
    switch (m_autoSelected) {
      case "1":
        m_autonomousCommand=m_autonomousCommand1;
        break;
      case "2":
        m_autonomousCommand=m_autonomousCommand2;
        break;
      case "3":
        m_autonomousCommand=m_autonomousCommand3;
        break;
      case "4":
        m_autonomousCommand=m_autonomousCommand4;
        break;
      case "5":
        m_autonomousCommand=m_autonomousCommand5;
        break;
      case "6":
        m_autonomousCommand=m_autonomousCommand6;
        break;  
      case "7":
        m_autonomousCommand=m_autonomousCommand7;
        break;          
      default:
      m_autonomousCommand=m_autonomousCommand1;
        break;
    }
    m_autonomousCommand.schedule();    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
//    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.reEnableGyro();
    m_robotContainer.setPIDslot(0);  // use the teleop PID gains for teleop 
    m_robotContainer.setAutoFlag(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
