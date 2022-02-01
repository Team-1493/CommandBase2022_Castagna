package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sensors.BallFollowCamera;

public class BallFollowInterface extends SubsystemBase{
    SwerveDriveSystem sds; 
    double currentHeading;
    double vmax=2.5;
    double vmin=0.5;
    double kp_ball=0.02;
    BallFollowCamera cam;

 // Constrcutor 
 public BallFollowInterface(SwerveDriveSystem m_sds){
    sds=m_sds;
    SmartDashboard.putNumber("ball kp", kp_ball);
    SmartDashboard.putNumber("ball vmax", vmax);
    SmartDashboard.putNumber("ball vmin", vmin);   
    SmartDashboard.putNumber("ball error", 0);
    SmartDashboard.putNumber("distance1", 0);
}    


public  void driveToTartget() {


    double angle;
    double centerX = SmartDashboard.getNumber("ball centerX",0.01);
    angle= 30*(centerX-240)/240*Math.PI/180.+ sds.heading;
  
    double centerY = SmartDashboard.getNumber("ball centerY",0.01);
    double distance = 9.88*Math.exp(464/(centerY+0.01));
    double target=40;
    double error = distance-target;
    double scale=1;
    SmartDashboard.putNumber("ball error", error);
    SmartDashboard.putNumber("distance1", distance);

        vmax=SmartDashboard.getNumber("ball vmax", 1.5);
        vmin=SmartDashboard.getNumber("ball vmin", 0.25);
        kp_ball=SmartDashboard.getNumber("ball kp", 0.01);
    
    if (Math.abs(error)>1){

        if(Math.abs(error)*kp_ball>vmax){
            scale=vmax/( Math.abs(error)*kp_ball);
        }

        double vx=(error)*Math.sin(angle)*kp_ball;
        double vy=(error)*Math.cos(angle)*kp_ball;
        vx=vx*scale;
        vy=vy*scale;
        SmartDashboard.putNumber("ball_vx", vx);
        SmartDashboard.putNumber("ball_vy", vy);
        

//        sds.setMotors(new double[] {0, 0,angle, 3});
        sds.setMotors(new double[] {vx, vy,angle, 3});
    }
    else sds.setMotors(new double[] {0, 0,angle, 3});
}



}