package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraInterface extends SubsystemBase{
    SwerveDriveSystem swerveDriveSystem; 
    double currentHeading;
    double vmax=0.3;
    private Tables datatable; 
    MedianFilter filter = new MedianFilter(5);

 // Constrcutor 
 public CameraInterface(SwerveDriveSystem sds){
    swerveDriveSystem=sds;
}    


public double[] driveToTartget(double[] prev_values) {
    double vx,vy,omega;

    // get the data you need from the raspberry pi over the network table
    // you can add any other data items that the raspberry pi might write to the table
    double average_length = SmartDashboard.getNumber("average_length",0);
    double gap = SmartDashboard.getNumber("gap",0);
    double slope_x = SmartDashboard.getNumber("slope_x", 0);
    double slope_y = SmartDashboard.getNumber("slope_y", 0);
    double average_length_filter=filter.calculate(average_length); 
    double error = 60 - average_length_filter ;


    //  Get the current robot heading in degrees if you need it (delete if not needed)
   // currentHeading=swerveDriveSystem.getHeading();
    
    // calculating vx, vy and headingset using the pid constants

    vx= -((Constants.ar_kp_vx * slope_x) + (Constants.ar_ki_vx*(slope_x - prev_values[0])));
    vy= (Constants.ar_kp_vy * error) + (Constants.ar_ki_vy*(error - prev_values[1]));
    omega = (Constants.ar_kp_hs * gap) + (Constants.ar_ki_hs*(gap - prev_values[2]));
    SmartDashboard.putNumber("cam_vx", vx);
    SmartDashboard.putNumber("cam_vy", vy);
    SmartDashboard.putNumber("cam_omega", omega);
    SmartDashboard.putNumber("cam_error", error);
    SmartDashboard.putNumber("average_length_filter", average_length_filter);
   // if (average_length==0)System.out.println(average_length);    
    if(vy>vmax)vy=vmax;
    if(vy<-vmax)vy=-vmax;
    if(vx>vmax)vx=vmax;
    if(vx<-vmax)vx=-vmax;
/*
    if (average_length == 0){
        vx = 0;
        vy = 0;
        omega = 0;
    }
*/
    // set the heading set to an angle in number of revolutions

    // set the desired robot speed and heading
    swerveDriveSystem.setMotors(new double[] {vx, vy, omega, 1});

    prev_values[0] = slope_x; // previous slope x
    prev_values[1] = error;   // previous error
    prev_values[2] = gap;     // previous gap
    
    return prev_values;
}



}