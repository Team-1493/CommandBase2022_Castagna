package frc.robot.Sensors;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
   Limelight(){
     
   }

   public double[] getData(){
     double [] data = {ta.getDouble(0.0),ty.getDouble(0.0),tx.getDouble(0.0)};
     SmartDashboard.putNumber("ta", data[0]);
     SmartDashboard.putNumber("ty", data[1]);
     SmartDashboard.putNumber("tx", data[2]);

     return data;
   }
}