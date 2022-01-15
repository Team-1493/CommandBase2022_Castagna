package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Utilities.CustomSwerveControllorCommand;
import frc.robot.Utilities.Util;
//import frc.robot.subsystems.PathPlannerTrajectory.PathPlannerState;
import static frc.robot.Constants.Constants_Swerve.*;


public class TrajectoryFollower extends SubsystemBase {
     Trajectory trajectory;
     SwerveDriveSystem sds;   
     double vm=5,am=5;
     double finalHeading=-90;

   // Constrcutor 
  public TrajectoryFollower(SwerveDriveSystem m_sds){
    sds=m_sds;    

  }


  //  returns a follow trajectory command
  public SequentialCommandGroup getFollowTrajCommand(){
        
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(vm), 
            Units.feetToMeters(am)).setKinematics(m_kinematics);

    Pose2d startPose = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0),
            Rotation2d.fromDegrees(0));
    Pose2d endPose = new Pose2d(Units.feetToMeters(10), Units.feetToMeters(0),
            Rotation2d.fromDegrees(finalHeading));

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(0.1), Units.feetToMeters(0)));

    trajectory = TrajectoryGenerator.generateTrajectory(startPose,interiorWaypoints,endPose,config);
    double trajtime=trajectory.getTotalTimeSeconds();
    SmartDashboard.putNumber("traj time", trajectory.getTotalTimeSeconds());
    
    PathPlannerTrajectory traj2  = PathPlanner.loadPath("Path1", 8, 5);
    trajtime=traj2.getTotalTimeSeconds();

    // Print the holonomic rotation at the sampled time
    int i=0;
    System.out.println("traj2");
    System.out.println("time,x,y,rot,hol,angV,angA,R,v,a");
    while(i<21){
        PathPlannerState state = (PathPlannerState) traj2.sample(i*trajtime/20);

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
        kP_rotate, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

CustomSwerveControllorCommand swerveControllerCommand =
    new CustomSwerveControllorCommand(
        traj2,
        sds::getPose, // Functional interface to feed supplier
        m_kinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        () -> ((PathPlannerState) ( ((PathPlannerTrajectory)traj2).getStates().get(1) )).holonomicRotation,
        sds::setModuleStates,
        sds);

// Reset odometry to the starting pose of the trajectory.
sds.resetOdometry(trajectory.getInitialPose());

// Run path following command, then stop at the end.
return swerveControllerCommand.andThen(() -> sds.setMotors(new double[] {0, 0, Util.toRadians(finalHeading), 3}));
}
}

  