package frc.robot.subsystems;


import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Utilities.CustomSwerveControllorCommand;
import frc.robot.Utilities.Util;
import frc.robot.commands.ResetPose;
import frc.robot.commands.IntakeShooter.IntakeBall;
import frc.robot.commands.IntakeShooter.IntakeFirstBallAuto;
import frc.robot.commands.IntakeShooter.ShootBallAuto;
import frc.robot.commands.LimelightFollowing.LimelightAlign;
import frc.robot.commands.IntakeShooter.ShootBall;


public class AutoGenerator extends SubsystemBase {
     SwerveDriveSystem sds;   
     private Shooter shooter;
     double finalHeading=90;
     private double kMaxAngularSpeedRadiansPerSecond = Math.PI;
     private double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

     private double kPXController = 1;
     private double kPYController = 1;
     private IntakeConveyor intake;

 // Constraint for the motion profilied robot angle controller
     private  TrapezoidProfile.Constraints kThetaControllerConstraints =
         new TrapezoidProfile.Constraints(
         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

   // Constrcutor 
  public AutoGenerator(SwerveDriveSystem m_sds, IntakeConveyor m_intake, Shooter m_shooter){
    sds=m_sds;
    intake = m_intake;
    shooter=m_shooter;
    SmartDashboard.putNumber("Trajectory kP_x", kPXController);    
    SmartDashboard.putNumber("Trajectory kP_y", kPYController);
    SmartDashboard.putNumber("Trajectory maxRotVel", kMaxAngularSpeedRadiansPerSecond);
    SmartDashboard.putNumber("Trajectory maxRotAcc", kMaxAngularSpeedRadiansPerSecondSquared);
  }


  //  returns a command sequence for a two-ball auto
  public SequentialCommandGroup getAuto1(){    
      // create as many trajectories as needed.  Need the initial pose only for the first trajectory!
    PathPlannerTrajectory traj1  = PathPlanner.loadPath("Path1", 0.5 ,1); 
    Pose2d initialPose = traj1.getInitialPose();
    CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);
    SequentialCommandGroup commandGroup = 
    new SequentialCommandGroup(
            new ResetPose(sds, initialPose), 
            new ParallelCommandGroup(
            new IntakeFirstBallAuto(intake),
            cscc1),
        new LimelightAlign(sds),
        new ShootBallAuto(intake, shooter,1)
        );
return commandGroup;
//    PathPlannerState endstate=  traj1.getEndState();
//    finalHeading=(endstate.holonomicRotation).getDegrees();
//return PathCommands.andThen(() -> sds.setMotors(new double[] {0, 0, Util.toRadians(0), 3}));
}


  //  returns a command sequence for a 4 ball auto **need to modify **
  public SequentialCommandGroup getAuto2(){    
    // create as many trajectories as needed.  Need the initial pose only for the first trajectory!
  PathPlannerTrajectory traj1  = PathPlanner.loadPath("Path1", 0.5 ,1); 
  Pose2d initialPose = traj1.getInitialPose();
  CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);
  SequentialCommandGroup commandGroup = 
  new SequentialCommandGroup(
          new ResetPose(sds, initialPose), 
          new ParallelCommandGroup(
          new IntakeFirstBallAuto(intake),
          cscc1),
      new LimelightAlign(sds),
      new ShootBallAuto(intake, shooter,1)
      );
return commandGroup;
}




//  method to get a swervecontrollercommand to follow a trajectorty
public CustomSwerveControllorCommand getSwerveControllerCommand(PathPlannerTrajectory traj){
    CustomSwerveControllorCommand cscc;

    var thetaController =
    new ProfiledPIDController(
        SwerveDriveSystem.kP_rotate, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    cscc=new CustomSwerveControllorCommand(
        traj,
        sds::getPose, // Functional interface to feed supplier
        SwerveDriveSystem.m_kinematics,

        // Position controllers
        new PIDController(kPXController, 0, 0),
        new PIDController(kPYController, 0, 0),
        thetaController,
        () -> ((PathPlannerState) ( ((PathPlannerTrajectory)traj).getStates().get(1) )).holonomicRotation,
        sds::setModuleStates,
        sds);
    return cscc;
}












public void updateConstants(){
    kPXController=SmartDashboard.getNumber("Trajectory kP_x", kPXController);    
    kPYController=SmartDashboard.getNumber("Trajectory kP_y", kPYController);
    kMaxAngularSpeedRadiansPerSecond =
         SmartDashboard.getNumber("Trajectory maxRotVel", kMaxAngularSpeedRadiansPerSecond);
    kMaxAngularSpeedRadiansPerSecondSquared = 
        SmartDashboard.getNumber("Trajectory maxRotAcc", kMaxAngularSpeedRadiansPerSecondSquared);
    kThetaControllerConstraints =new TrapezoidProfile.Constraints(
         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
}




}

//.andThen(() -> sds.setMotors(new double[] {0, 0, Util.toRadians(finalHeading), 3}))  