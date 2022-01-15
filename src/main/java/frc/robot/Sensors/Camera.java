package frc.robot.Sensors;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;



public class Camera extends SubsystemBase{
 
 // Constrcutor 
 public Camera(){
 
    CameraServer.startAutomaticCapture(0);

}    



}