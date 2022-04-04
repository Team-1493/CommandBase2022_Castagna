package frc.robot.subsystems;



import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Utilities.CustomSwerveControllorCommand;
import frc.robot.commands.ResetPose;
import frc.robot.commands.IntakeShooter.IntakeFirstBallAuto;
import frc.robot.commands.IntakeShooter.LowerIntake;
import frc.robot.commands.IntakeShooter.ShootBallAuto;
import frc.robot.commands.Rotate.AlignWithField;

public class AutoGenerator extends SubsystemBase {
     private SwerveDriveSystem sds;   
     private Shooter shooter;
     private double kMaxAngularSpeedRadiansPerSecond = 10;
     private double kMaxAngularSpeedRadiansPerSecondSquared = 30;

     private double kPXController = 7;
     private double kPYController = 7;
     private double kDXController = 0;
     private double kDYController = 0;
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
    thetaController = new ProfiledPIDController(12, 0, 0.1, 
        kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController= new PIDController(kPXController, 0, kDXController);
    yController = new PIDController(kPYController, 0, kDYController);



    
    SmartDashboard.putNumber("Trajectory kP_x", kPXController);    
    SmartDashboard.putNumber("Trajectory kP_y", kPYController);
    SmartDashboard.putNumber("Trajectory kD_x", kDXController);    
    SmartDashboard.putNumber("Trajectory kD_y", kDYController);

    SmartDashboard.putNumber("Trajectory maxRotVel", kMaxAngularSpeedRadiansPerSecond);
    SmartDashboard.putNumber("Trajectory maxRotAcc", kMaxAngularSpeedRadiansPerSecondSquared);
  }




  //  returns a command sequence for a 2-ball auto
  public SequentialCommandGroup getAuto1(){    
    PathPlannerTrajectory traj1  = PathPlanner.loadPath("5b Path1", 2 ,2); 
    Pose2d initialPose1 = traj1.getInitialPose();
    CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);
  
    PathPlannerTrajectory traj2  = PathPlanner.loadPath("2b Path2a", 2 ,2); 
    CustomSwerveControllorCommand cscc2=getSwerveControllerCommand(traj2);
  
    PathPlannerTrajectory traj3  = PathPlanner.loadPath("2b Path3a", 2 ,2); 
    CustomSwerveControllorCommand cscc3=getSwerveControllerCommand(traj3);

    PathPlannerTrajectory traj4  = PathPlanner.loadPath("2b Path4a", 2 ,2); 
    CustomSwerveControllorCommand cscc4=getSwerveControllerCommand(traj4);
  
  
  SequentialCommandGroup commandGroup = 
  new SequentialCommandGroup(
    new ResetPose(sds, initialPose1), 
    new InstantCommand( ()->resetControllers()),
    new LowerIntake(intake).andThen(new WaitCommand(0.3)),
    (cscc1.andThen(new WaitCommand(.3))).deadlineWith( new IntakeFirstBallAuto(intake)) ,
    new InstantCommand(()-> sds.allStop()),

    new ShootBallAuto(intake, shooter,3,1750),
    new InstantCommand( ()->resetControllers()),
    new InstantCommand( ()->sds.allStop() ),

    cscc2.deadlineWith( new IntakeFirstBallAuto(intake)) ,
    new InstantCommand( ()->resetControllers()),
    new InstantCommand(()-> sds.allStop()),

    cscc3,
    new InstantCommand( ()->resetControllers()),
    new InstantCommand(()-> sds.allStop()),
    new ShootBallAuto(intake, shooter,3,1500),
    
    cscc4,
    new InstantCommand(()-> sds.allStop()),
    new InstantCommand( ()->resetControllers()),
    new InstantCommand( ()-> sds.setHeading(0))
  );
return commandGroup;
}


  //  returns a command sequence for a 2 ball auto on far side
  public SequentialCommandGroup getAuto2(){    
  PathPlannerTrajectory traj1  = PathPlanner.loadPath("2b Path1", 2 ,2); 
  Pose2d initialPose1 = traj1.getInitialPose();
  CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);

  PathPlannerTrajectory traj2  = PathPlanner.loadPath("2b Path2", 2 ,2); 
  CustomSwerveControllorCommand cscc2=getSwerveControllerCommand(traj2);

  PathPlannerTrajectory traj3  = PathPlanner.loadPath("2b Path3", 2 ,2); 
  CustomSwerveControllorCommand cscc3=getSwerveControllerCommand(traj3);

  PathPlannerTrajectory traj4  = PathPlanner.loadPath("2b Path4", 2 ,2); 
  CustomSwerveControllorCommand cscc4=getSwerveControllerCommand(traj4);



  SequentialCommandGroup commandGroup = 
  new SequentialCommandGroup(
      new ResetPose(sds, initialPose1), 
      new InstantCommand( ()->resetControllers()),
      new LowerIntake(intake).andThen(new WaitCommand(0.3)),
      (cscc1.andThen(new WaitCommand(.3))).deadlineWith( new IntakeFirstBallAuto(intake)) ,
      new InstantCommand(()-> sds.allStop()),

      new ShootBallAuto(intake, shooter,3,1750),
      new InstantCommand( ()->resetControllers()),
      new InstantCommand( ()->sds.allStop() ),

      cscc2.deadlineWith( new IntakeFirstBallAuto(intake)) ,
      new InstantCommand( ()->resetControllers()),
      new InstantCommand(()-> sds.allStop()),

      cscc3,
      new InstantCommand( ()->resetControllers()),
      new InstantCommand(()-> sds.allStop()),
      new ShootBallAuto(intake, shooter,3,1000),

      cscc4,
      new InstantCommand(()-> sds.allStop()),
      new InstantCommand( ()->resetControllers()),
      new InstantCommand( ()-> sds.setHeading(0))
    );
  return commandGroup;
}


  //  returns a command sequence for a 5 ball auto
  public SequentialCommandGroup getAuto3(){    
  PathPlannerTrajectory traj1  = PathPlanner.loadPath("5b Path1", 2 ,2); //3,3 
  Pose2d initialPose1 = traj1.getInitialPose();
  CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);

  PathPlannerTrajectory traj2  = PathPlanner.loadPath("5b Path2", 2 ,2);// 3,3.5 
  CustomSwerveControllorCommand cscc2=getSwerveControllerCommand(traj2);

  PathPlannerTrajectory traj4  = PathPlanner.loadPath("5b Path4", 2 ,2);//3,3.5 
  CustomSwerveControllorCommand cscc4=getSwerveControllerCommand(traj4);

  PathPlannerTrajectory traj5  = PathPlanner.loadPath("5b Path5", 2 ,2);//4,4 
  CustomSwerveControllorCommand cscc5=getSwerveControllerCommand(traj5);

  PathPlannerTrajectory traj6  = PathPlanner.loadPath("5b Path6", 2 ,2);//4,6 
  CustomSwerveControllorCommand cscc6=getSwerveControllerCommand(traj6);

  SequentialCommandGroup commandGroup = 
  new SequentialCommandGroup(
    new ResetPose(sds, initialPose1), 
    new InstantCommand( ()->resetControllers()),

    new LowerIntake(intake).andThen(new WaitCommand(0.2)),
    (cscc1.andThen(new WaitCommand(.25))).deadlineWith( new IntakeFirstBallAuto(intake)) ,
    new InstantCommand(()-> sds.allStop()),
    new ShootBallAuto(intake, shooter, 3, 1740),

    (cscc2.andThen(new WaitCommand(0.25) )).deadlineWith(new IntakeFirstBallAuto(intake)),
    new InstantCommand( ()->resetControllers()),
    new InstantCommand(()-> sds.allStop()),
    new ShootBallAuto(intake, shooter,3, 1835),
    new InstantCommand( ()->resetControllers()),

   
    (cscc4.andThen(new WaitCommand(0.5))).deadlineWith(new IntakeFirstBallAuto(intake)),

    new InstantCommand( ()->resetControllers()),
    cscc5,
    new InstantCommand(()-> sds.allStop()),
    new ShootBallAuto(intake, shooter,3,1730),

    cscc6,
    new InstantCommand(()-> sds.allStop()),
    new InstantCommand( ()->resetControllers()),
    new InstantCommand( ()-> sds.setHeading(0))
    );
return commandGroup;
}



  //  returns a command sequence for a 5 ball auto on red side
  public SequentialCommandGroup getAuto5(){    
    PathPlannerTrajectory traj1  = PathPlanner.loadPath("5b Path1 red", 2 ,2); // 3, 3
    Pose2d initialPose1 = traj1.getInitialPose();
    CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);
  
    PathPlannerTrajectory traj2  = PathPlanner.loadPath("5b Path2 red", 3 ,2); // 3, 3.5
    CustomSwerveControllorCommand cscc2=getSwerveControllerCommand(traj2);
  
    PathPlannerTrajectory traj4  = PathPlanner.loadPath("5b Path4 red", 3 ,2); // 3, 3.5 
    CustomSwerveControllorCommand cscc4=getSwerveControllerCommand(traj4);
  
    PathPlannerTrajectory traj5  = PathPlanner.loadPath("5b Path5 red", 3 ,2); // 4, 4  
    CustomSwerveControllorCommand cscc5=getSwerveControllerCommand(traj5);
  
    PathPlannerTrajectory traj6  = PathPlanner.loadPath("5b Path6 red", 3 ,2);  // 3, 3 
    CustomSwerveControllorCommand cscc6=getSwerveControllerCommand(traj6);
  
    SequentialCommandGroup commandGroup = 
    new SequentialCommandGroup(
      new ResetPose(sds, initialPose1), 
      new InstantCommand( ()->resetControllers()),
  
      new LowerIntake(intake).andThen(new WaitCommand(0.2)),
      cscc1.deadlineWith( new IntakeFirstBallAuto(intake)) ,
      new InstantCommand(()-> sds.allStop()),
      new ShootBallAuto(intake, shooter, 3, 1730),
      new InstantCommand( ()->resetControllers()),
  
  
      cscc2.deadlineWith(new IntakeFirstBallAuto(intake).withInterrupt(()->intake.ballAtTop)),
      new InstantCommand( ()->resetControllers()),
      new InstantCommand(()-> sds.allStop()),
      new ShootBallAuto(intake, shooter,3, 1775),
     
     
      (cscc4.andThen(new WaitCommand(0.5))).deadlineWith(new IntakeFirstBallAuto(intake)),
  
      new InstantCommand( ()->resetControllers()),
      cscc5,
      new InstantCommand(()-> sds.allStop()),
      new ShootBallAuto(intake, shooter,3,1730),
  
      cscc6,
      new InstantCommand(()-> sds.allStop()),
      new InstantCommand( ()->resetControllers()),
      new InstantCommand( ()-> sds.setHeading(0))
      );
  return commandGroup;
  }
  
  
  

  //  returns a command sequence for a short test  auto
  public SequentialCommandGroup getAuto6(){    
    PathPlannerTrajectory traj1  = PathPlanner.loadPath("TestPath20ft", 2 ,2); 
    Pose2d initialPose1 = traj1.getInitialPose();
    CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);
  
  
  SequentialCommandGroup commandGroup = 
  new SequentialCommandGroup(
    new ResetPose(sds, initialPose1), 
    new InstantCommand( ()->resetControllers()),
    cscc1,
    new InstantCommand(()-> sds.allStop())

  );
return commandGroup;
}

  //  returns a command sequence for a 4 ball auto
  public SequentialCommandGroup getAuto4(){    
    PathPlannerTrajectory traj1  = PathPlanner.loadPath("4b Path1", 2.5 ,2.5); 
    Pose2d initialPose1 = traj1.getInitialPose();
    CustomSwerveControllorCommand cscc1=getSwerveControllerCommand(traj1);
  
    PathPlannerTrajectory traj2  = PathPlanner.loadPath("4b Path2", 2.5 ,3); 
    CustomSwerveControllorCommand cscc2=getSwerveControllerCommand(traj2);
  
    PathPlannerTrajectory traj3  = PathPlanner.loadPath("4b Path3", 2.5 ,3); 
    CustomSwerveControllorCommand cscc3=getSwerveControllerCommand(traj3);
  
  
    SequentialCommandGroup commandGroup = 
    new SequentialCommandGroup(
      new ResetPose(sds, initialPose1), 
      new InstantCommand( ()->resetControllers()),
  
      new LowerIntake(intake).andThen(new WaitCommand(0.2)),
      cscc1.deadlineWith( new IntakeFirstBallAuto(intake)) ,
      new InstantCommand(()-> sds.allStop()),
      new ShootBallAuto(intake, shooter, 3, 1750),
      new InstantCommand( ()->resetControllers()),
  
       
      (cscc2.andThen(new WaitCommand(1.0))).deadlineWith(new IntakeFirstBallAuto(intake)),
  
      new InstantCommand( ()->resetControllers()),
      cscc3,
      new InstantCommand(()-> sds.allStop()),
      new ShootBallAuto(intake, shooter,3,1750),
      new InstantCommand( ()-> sds.setHeading(-155))
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
        sds::printTrajectoryPose,
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
    kDXController=SmartDashboard.getNumber("Trajectory kD_x", kDXController);    
    kDYController=SmartDashboard.getNumber("Trajectory kD_y", kDYController);
    kMaxAngularSpeedRadiansPerSecond =
         SmartDashboard.getNumber("Trajectory maxRotVel", kMaxAngularSpeedRadiansPerSecond);
    kMaxAngularSpeedRadiansPerSecondSquared = 
        SmartDashboard.getNumber("Trajectory maxRotAcc", kMaxAngularSpeedRadiansPerSecondSquared);
    kThetaControllerConstraints =new TrapezoidProfile.Constraints(
         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
         thetaController = new ProfiledPIDController(SwerveDriveSystem.kP_rotate, 0, SwerveDriveSystem.kD_rotate, 
         kThetaControllerConstraints);
     thetaController.enableContinuousInput(-Math.PI, Math.PI);
     xController= new PIDController(kPXController, 0, kDXController);
     yController = new PIDController(kPYController, 0, kDYController);
}




}

