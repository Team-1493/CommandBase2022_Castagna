package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightInterface extends SubsystemBase{
   
    public NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public NetworkTable limelight= inst.getTable("limelight");
    public NetworkTableEntry txEntry = limelight.getEntry("tx");
    public NetworkTableEntry tvEntry = limelight.getEntry("tv");
    private SwerveDriveSystem sds;
 // Constrcutor 
    public LimelightInterface(SwerveDriveSystem m_sds){
        sds=m_sds;
    }    

    
    
        public  void turnToTarget() {
        double angle=-txEntry.getDouble(0)/5 - 5;
        double angleRadians=Math.toRadians(angle);
            
        if(tvEntry.getDouble(0)==1)
            sds.setMotors(new double[] {0, 0,angleRadians, 3});    
        }

    }



