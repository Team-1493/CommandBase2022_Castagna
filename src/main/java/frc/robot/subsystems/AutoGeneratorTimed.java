package frc.robot.subsystems;


import java.nio.file.Path; 

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.CustomSwerveControllorCommand;
import frc.robot.commands.ResetPose;
import frc.robot.commands.IntakeShooter.IntakeFirstBallAuto;
import frc.robot.commands.IntakeShooter.ShootBallAuto;
import frc.robot.commands.LimelightFollowing.LimelightAlign;


public class AutoGeneratorTimed extends SubsystemBase {
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
  public AutoGeneratorTimed(SwerveDriveSystem m_sds, IntakeConveyor m_intake, Shooter m_shooter){
    sds=m_sds;
    intake = m_intake;
    shooter=m_shooter;
    SmartDashboard.putNumber("Trajectory kP_x", kPXController);    
    SmartDashboard.putNumber("Trajectory kP_y", kPYController);
    SmartDashboard.putNumber("Trajectory maxRotVel", kMaxAngularSpeedRadiansPerSecond);
    SmartDashboard.putNumber("Trajectory maxRotAcc", kMaxAngularSpeedRadiansPerSecondSquared);
  }




  //  returns a command sequence for a Timed 4-ball auto

  public SequentialCommandGroup getAuto5(){    
  PathPlannerTrajectory traj1  = PathPlanner.loadPath("Timed4Ball", 1.5 ,4); 
  Pose2d initialPose = traj1.getInitialPose();
  CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);

  SequentialCommandGroup commandGroup = 
  //Goes to first ball, picks it up, and shoots it
  new SequentialCommandGroup(
    new ResetPose(sds, initialPose), 
    new ParallelCommandGroup(
        new IntakeFirstBallAuto(intake),
        cscc1
    )
  );
return commandGroup;
//    PathPlannerState endstate=  traj1.getEndState();
//    finalHeading=(endstate.holonomicRotation).getDegrees();
//return PathCommands.andThen(() -> sds.setMotors(new double[] {0, 0, Util.toRadians(0), 3}));
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