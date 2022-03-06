package frc.robot.subsystems;
import java.text.DecimalFormat;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//  All this class does is write the list entries to be logged 
// and the time value to the smartdashboard

public class Tables extends SubsystemBase {  
    static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    static NetworkTable table= inst.getTable("Datatable");
    NetworkTableEntry[] tableEntry = new NetworkTableEntry[20];
    static NetworkTableEntry timeEntry;
    static NetworkTableEntry entryNames = table.getEntry("entry names");
    String[] names;
    String heading;
    DecimalFormat df = new DecimalFormat("###.####");
    int numberNames;


//String entryNamesList = "FL Opt Angle,FL Tpos,FL Tvel,FL SP rot,FL SP RPM,FL TmotorOut,heading,gyro temp,FL Dvel,heading";
String entryNamesList = "FR SP RPM,FR Dvel";


public Tables(){
    SmartDashboard.putString("entry names", entryNamesList);
    entryNames.setString(entryNamesList);
    timeEntry=table.getEntry("time");
    timeEntry.setDouble(Timer.getFPGATimestamp());

    // don't think we nned this anymore
    int i=0;
    while(i<numberNames){
        tableEntry[i]=table.getEntry(names[i]);
        i++;
    }
}  

public void putNumber(String key, double num ){    
    NetworkTableEntry entry = table.getEntry(key);
    entry.setNumber(num);
    SmartDashboard.putNumber(key, num);
}

public void update(){
    timeEntry.setDouble(Timer.getFPGATimestamp());
    inst.flush();    
}
}
