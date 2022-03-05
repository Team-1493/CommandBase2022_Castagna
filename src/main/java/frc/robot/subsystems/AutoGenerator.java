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


public class AutoGenerator extends SubsystemBase {
     private SwerveDriveSystem sds;   
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

    public static ProfiledPIDController thetaController;
    public static PIDController xController,yController;


   // Constrcutor 
  public AutoGenerator(SwerveDriveSystem m_sds, IntakeConveyor m_intake, Shooter m_shooter){
    sds=m_sds;
    intake = m_intake;
    shooter=m_shooter;
    thetaController = new ProfiledPIDController(SwerveDriveSystem.kP_rotate, 0, 0, 
        kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController= new PIDController(kPXController, 0, 0);
    yController = new PIDController(kPYController, 0, 0);


    
    SmartDashboard.putNumber("Trajectory kP_x", kPXController);    
    SmartDashboard.putNumber("Trajectory kP_y", kPYController);
    SmartDashboard.putNumber("Trajectory maxRotVel", kMaxAngularSpeedRadiansPerSecond);
    SmartDashboard.putNumber("Trajectory maxRotAcc", kMaxAngularSpeedRadiansPerSecondSquared);
  }




  //  returns a command sequence for a 2-ball auto
  public SequentialCommandGroup getAuto1(){    
    // create as many trajectories as needed.  Need the initial pose only for the first trajectory!
  PathPlannerTrajectory traj1  = PathPlanner.loadPath("Path1", 0.5 ,1); 
  Pose2d initialPose1 = traj1.getInitialPose();
  CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);
  PathPlannerTrajectory traj2  = PathPlanner.loadPath("Path1 Back", 0.5 ,1); 
  Pose2d initialPose2 = traj2.getInitialPose();

  CustomSwerveControllorCommand cscc2=getSwerveControllerCommand(traj2);
  SequentialCommandGroup commandGroup = 
  //Goes to first ball, picks it up, and shoots it
  new SequentialCommandGroup(
    new ResetPose(sds, initialPose1), 
    new InstantCommand( ()->resetControllers()),
    cscc1.alongWith(new IntakeFirstBallAuto(intake)),
  //new LimelightAlign(sds),
    new ShootBallAuto(intake, shooter,2),
    new ResetPose(sds, initialPose2), 
    new InstantCommand( ()->resetControllers()),
    cscc2,
    new InstantCommand( ()->resetControllers()),
    new InstantCommand( ()->sds.setMotors(new double[] {0, 0,sds.heading, 3}) )
  );
return commandGroup;
}



  //  returns a command sequence for a 4-ball auto
  public SequentialCommandGroup getAuto2(){    
      // create as many trajectories as needed.  Need the initial pose only for the first trajectory!
    PathPlannerTrajectory traj1  = PathPlanner.loadPath("Path1", 0.5 ,1); 
    Pose2d initialPose1 = traj1.getInitialPose();
    CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);

    PathPlannerTrajectory traj2  = PathPlanner.loadPath("Path2", 0.5 ,1); 
    Pose2d initialPose2 = traj2.getInitialPose();
    CustomSwerveControllorCommand cscc2=getSwerveControllerCommand(traj2);

    PathPlannerTrajectory traj3  = PathPlanner.loadPath("Path3 Alt Alt", 0.5 ,1); 
    Pose2d initialPose3 = traj3.getInitialPose();
    CustomSwerveControllorCommand cscc3=getSwerveControllerCommand(traj3);

    PathPlannerTrajectory traj4  = PathPlanner.loadPath("Path4 Alt Alt", 0.5 ,1);
    Pose2d initialPose4 = traj4.getInitialPose(); 
    CustomSwerveControllorCommand cscc4=getSwerveControllerCommand(traj4);

    SequentialCommandGroup commandGroup = 
    //Goes to first ball, picks it up, and shoots it
    new SequentialCommandGroup(
        new ResetPose(sds, initialPose1),
        new InstantCommand( ()->resetControllers()),
        new ParallelCommandGroup(
            new IntakeFirstBallAuto(intake),
            cscc1
        ),
    new LimelightAlign(sds),
    new ShootBallAuto(intake, shooter,1),
    //Goes to get next ball
    new ParallelCommandGroup(
        new IntakeFirstBallAuto(intake),
        new SequentialCommandGroup(
            new ResetPose(sds, initialPose2),
            new InstantCommand( ()->resetControllers()),
            cscc2,
            new ResetPose(sds, initialPose3),
            new InstantCommand( ()->resetControllers()),
            cscc3
        )
    ),
    new ResetPose(sds, initialPose4),
    new InstantCommand( ()->resetControllers()),
    cscc4,
    new ShootBallAuto(intake, shooter,1),
    new InstantCommand( ()->resetControllers()),
    new InstantCommand( ()->sds.setMotors(new double[] {0, 0,sds.heading, 3}) )
    );
return commandGroup;
}


  //  returns a command sequence for a 5 ball auto **need to modify **
  public SequentialCommandGroup getAuto3(){    
    // create as many trajectories as needed.  Need the initial pose only for the first trajectory!
  PathPlannerTrajectory traj1  = PathPlanner.loadPath("Path1", 0.5 ,1); 
  Pose2d initialPose1 = traj1.getInitialPose();
  CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);

  PathPlannerTrajectory traj2  = PathPlanner.loadPath("Path2", 0.5 ,1); 
  Pose2d initialPose2 = traj2.getInitialPose();
  CustomSwerveControllorCommand cscc2=getSwerveControllerCommand(traj2);

  PathPlannerTrajectory traj3  = PathPlanner.loadPath("Path3", 0.5 ,1);
  Pose2d initialPose3 = traj3.getInitialPose(); 
  CustomSwerveControllorCommand cscc3=getSwerveControllerCommand(traj3);

  PathPlannerTrajectory traj4  = PathPlanner.loadPath("Path4", 0.5 ,1); 
  Pose2d initialPose4 = traj4.getInitialPose();
  CustomSwerveControllorCommand cscc4=getSwerveControllerCommand(traj4);

  PathPlannerTrajectory traj5  = PathPlanner.loadPath("Path5", 0.5 ,1); 
  Pose2d initialPose5 = traj5.getInitialPose();
  CustomSwerveControllorCommand cscc5=getSwerveControllerCommand(traj5);

  SequentialCommandGroup commandGroup = 
  new SequentialCommandGroup(
    new ResetPose(sds, initialPose1), 
    new InstantCommand( ()->resetControllers()),
    new ParallelCommandGroup(
        new IntakeFirstBallAuto(intake),
        cscc1
    ),
    new LimelightAlign(sds), //need to either take out and do angle aiming in pathplanner(this is the better option) 
    //or compensate for it in the code
    new ShootBallAuto(intake, shooter,1),
    new ParallelCommandGroup(
        new IntakeFirstBallAuto(intake).withInterrupt(()->intake.ballAtTop),
        new ResetPose(sds, initialPose2), 
        new InstantCommand( ()->resetControllers()),
        cscc2
    ),
    new ResetPose(sds, initialPose3), 
    new InstantCommand( ()->resetControllers()),
    cscc3,
    new ShootBallAuto(intake, shooter,1),
    new ParallelCommandGroup(
        new IntakeFirstBallAuto(intake),
        new ResetPose(sds, initialPose4), 
        new InstantCommand( ()->resetControllers()),
        cscc4
    ),
    new ResetPose(sds, initialPose5), 
    new InstantCommand( ()->resetControllers()),
    cscc5,
    new ShootBallAuto(intake, shooter,1),
    new InstantCommand( ()->resetControllers()),
    new InstantCommand( ()->sds.setMotors(new double[] {0, 0,sds.heading, 3}) )
    );
return commandGroup;
}

public SequentialCommandGroup getAuto4(){    
    // create as many trajectories as needed.  Need the initial pose only for the first trajectory!
  PathPlannerTrajectory traj1  = PathPlanner.loadPath("Path1", 0.5 ,1); 
  Pose2d initialPose = traj1.getInitialPose();
  CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);

  PathPlannerTrajectory traj2  = PathPlanner.loadPath("Path1 Back", 0.5 ,1); 
  CustomSwerveControllorCommand cscc2=getSwerveControllerCommand(traj2);

  SequentialCommandGroup commandGroup = 
  //Goes to first ball, picks it up, and shoots it
  new SequentialCommandGroup(
      new ResetPose(sds, initialPose), 
      new ParallelCommandGroup(
          new IntakeFirstBallAuto(intake),
          cscc1
      ),
  //new LimelightAlign(sds),
  new ShootBallAuto(intake, shooter,2),
  cscc2
  );
return commandGroup;
}

//  method to get a swervecontrollercommand to follow a trajectorty
public CustomSwerveControllorCommand getSwerveControllerCommand(PathPlannerTrajectory traj){
    CustomSwerveControllorCommand cscc;
    cscc=new CustomSwerveControllorCommand(
        traj,
        sds::getPose, // Functional interface to feed supplier
        SwerveDriveSystem.m_kinematics,

        // Position controllers
        xController,
        yController,
        thetaController,
        () -> ((PathPlannerState) ( ((PathPlannerTrajectory)traj).getStates().get(1) )).holonomicRotation,
        sds::setModuleStates,
        sds);
    return cscc;
}



public void resetControllers(){
    xController.reset();
    yController.reset();
    thetaController.reset(sds.getPose().getRotation().getRadians());
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