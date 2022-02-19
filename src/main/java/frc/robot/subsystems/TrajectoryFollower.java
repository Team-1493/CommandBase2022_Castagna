package frc.robot.subsystems;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.CustomSwerveControllorCommand;
import frc.robot.Utilities.Util;


public class TrajectoryFollower extends SubsystemBase {
     SwerveDriveSystem sds;   
     double vm=5,am=5;
     double finalHeading=90;
     private double kMaxAngularSpeedRadiansPerSecond = Math.PI;
     private double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

     private double kPXController = 1;
     private double kPYController = 1;
     private double kPThetaController = 1;

 // Constraint for the motion profilied robot angle controller
     private final TrapezoidProfile.Constraints kThetaControllerConstraints =
         new TrapezoidProfile.Constraints(
         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

   // Constrcutor 
  public TrajectoryFollower(SwerveDriveSystem m_sds){
    sds=m_sds;    

  }

  //  returns a follow trajectory command
  public SequentialCommandGroup getFollowTrajCommand(){    
    PathPlannerTrajectory traj2  = PathPlanner.loadPath("Path1", 3, 3);
    double trajtime=traj2.getTotalTimeSeconds();
    SmartDashboard.putNumber("traj time", trajtime);

    // Print the holonomic rotation at the sampled time
    int i=0;
    System.out.println("traj2");
    System.out.println("time,x,y,rot,hol,angV,angA,R,v,a");
    while(i<101){
        PathPlannerState state = (PathPlannerState) traj2.sample(i*trajtime/100);

            System.out.println(state.timeSeconds+","+
            state.poseMeters.getX()+","+
            state.poseMeters.getY()+","+
            state.poseMeters.getRotation().getDegrees()+","+
            state.holonomicRotation.getDegrees()+","+
            state.angularVelocity.getDegrees()+","+
            state.angularAcceleration.getDegrees()+","+
            state.curvatureRadPerMeter+","+
            state.velocityMetersPerSecond+","+
            state.accelerationMetersPerSecondSq);

            i++;
    }


    var thetaController =
    new ProfiledPIDController(
        SwerveDriveSystem.kP_rotate, 0, 0, kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

CustomSwerveControllorCommand swerveControllerCommand =
    new CustomSwerveControllorCommand(
        traj2,
        sds::getPose, // Functional interface to feed supplier
        SwerveDriveSystem.m_kinematics,

        // Position controllers
        new PIDController(kPXController, 0, 0),
        new PIDController(kPYController, 0, 0),
        thetaController,
        () -> ((PathPlannerState) ( ((PathPlannerTrajectory)traj2).getStates().get(1) )).holonomicRotation,
        sds::setModuleStates,
        sds);

// Reset odometry to the starting pose of the trajectory.
sds.resetOdometry(traj2.getInitialPose());

// Run path following command, then stop at the end.
return swerveControllerCommand.andThen(() -> sds.setMotors(new double[] {0, 0, Util.toRadians(finalHeading), 3}));
}
}

  