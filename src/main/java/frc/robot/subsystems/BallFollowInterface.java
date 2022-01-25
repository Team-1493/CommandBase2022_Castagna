package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Sensors.BallFollowCamera;

public class BallFollowInterface extends SubsystemBase{
    SwerveDriveSystem sds; 
    double currentHeading;
    double vmax=2;
    double vmin=0.25;
    double kp_ball=0.01;
    BallFollowCamera cam;
    

 // Constrcutor 
 public BallFollowInterface(SwerveDriveSystem m_sds,BallFollowCamera m_cam){
    sds=m_sds;
    cam=m_cam;
}    


public  void driveToTartget() {
    double dist = cam.getDistance();
    double angle = cam.getAngle()+ sds.heading;
    double target=23;
    if (Math.abs(dist-target)>1){
        double vx=(dist-target)*Math.sin(angle)*kp_ball;
        double vy=(dist-target)*Math.cos(angle)*kp_ball;
        if(Math.abs(vy)>vmax) vy=vmax*Math.signum(vy);
        if(Math.abs(vx)>vmax) vx=vmax*Math.signum(vx);
        if(Math.abs(vy)<vmin) vy=vmin*Math.signum(vy);
        if(Math.abs(vx)<vmin) vx=vmin*Math.signum(vx);
        sds.setMotors(new double[] {vx, vy,angle, 3});
    }
    else sds.setMotors(new double[] {0, 0,angle, 3});
}



}