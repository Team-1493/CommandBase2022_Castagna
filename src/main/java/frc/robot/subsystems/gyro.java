
package frc.robot.subsystems;

//imports pigeon
import com.ctre.phoenix.sensors.PigeonIMU;

//imports Rotation2d
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Sensors.Pigeon;
import frc.robot.commands.ResetGyro;

//creates instance of pigeon
public class gyro
{
    public double gyro() 
    {
        //resets to default values
        PigeonIMU pigeon = new PigeonIMU(20);
        pigeon.configFactoryDefault();
            pigeon.setYaw(0);
        //gets the yaw angle
        double[] ypr_deg = new double[3];
        pigeon.getYawPitchRoll(ypr_deg);
                double yawAngle=ypr_deg[0];
        //gets direction of robot (-180 - 180)
        int ang;
        double outputval;
        double inputval;
        ang=360;
        inputval=yawAngle;
        outputval= inputval % ang;
        double absAng= outputval - 180;
        return absAng;
        //returns yaw angle in degrees (0-360)
        return outputval;
        //gets gyro temp
            double temp=pigeon.getTemp();
            return temp;
        //Returns yaw angle in radians
        double absAngRad = (absAng*3.14)/180;
        return absAngRad;
        //returns 2d object
        public Rotation2d getRotation2d();
                return new Rotation2d(getAngleRadians());
        //resets the yaw angle to zero 
        resetGyro(pigeon);
    }

    private void resetGyro(PigeonIMU pigeon) {
        pigeon.configFactoryDefault();
            pigeon.setYaw(0);
    }
}