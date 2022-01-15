package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Util {
    static private double Pi = Math.PI;
    static  private double twoPi=2*Pi;
    static  private double threePiover2=3*Pi/2;
    static  private double Piover2=Pi/2;

    static public  SwerveModuleState optimize(SwerveModuleState sms, double currentAngle){
        double angleTarget=sms.angle.getRadians();
        double anglediff=angleTarget-currentAngle%twoPi;
    //    SmartDashboard.putNumber("angleTarget", angleTarget);
    //    SmartDashboard.putNumber("currentAngle", currentAngle);
    //    SmartDashboard.putNumber("angleDiff", anglediff);
        double angleChange=0;
        double optimizedAngle;
        int rev=1;
        if (anglediff>threePiover2) anglediff=anglediff-twoPi;
        if (anglediff<-threePiover2) anglediff=anglediff+twoPi;
    
        if (anglediff >Piover2 && anglediff < threePiover2 ) {
            angleChange=anglediff-Pi;
            rev=-1;}
        else if (anglediff <-Piover2 && anglediff > -threePiover2 ) {
            angleChange=anglediff+Pi;
            rev=-1;}    
        else angleChange=anglediff;    
        optimizedAngle=angleChange+currentAngle;
        double speed = sms.speedMetersPerSecond*rev;    
    
        return new SwerveModuleState(
            speed, new Rotation2d(optimizedAngle));
    }


    static public double toRadians(double degrees){
        return degrees*Math.PI/180.; 
    }

    static public double toDegrees(double radians){
        return radians*180./Math.PI; 
    }

}
