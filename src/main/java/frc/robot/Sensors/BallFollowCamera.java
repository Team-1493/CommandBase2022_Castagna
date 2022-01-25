package frc.robot.Sensors;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;

public class BallFollowCamera {
    static CvSink cvSink;
    static CvSource outputStream;
    static String config;
    static int raw_Brightness=1,Brightness=1,raw_Exposure=1,Exposure=1;
    static double centerX=-1,centerY=-1;
    static double angle,distance;
    static double rectTLy, rectBRy, rectTLx,rectBRx;
    static BlueBallPipeline bluePipe = new BlueBallPipeline();

public BallFollowCamera(){
    getVideo();
}

public static void getVideo() { 
// this is an inline thread that just runs repeatedly
// you could add a reference to a static variable within the thread that
// will turn on and off the image processing, change pipeline, etc.            
    new Thread(() -> {
        double maxarea=-1;
        double area=0;
        int maxareaContour=0;           
        int i=0;
        Boolean updateCamPrev=false;
        SmartDashboard.putBoolean("updateCam", false);
        SmartDashboard.putNumber("Exposure", Exposure);
        SmartDashboard.putNumber("raw_Exposure", raw_Exposure);
        SmartDashboard.putNumber("Brightness", Brightness);
        SmartDashboard.putNumber("raw_Brightness", raw_Brightness);
        String config = "{\"properties\":[";
        config=config+ "{\"name\":\"raw_Exposure\",\"value\":"+raw_Exposure+"},";
        config=config+ "{\"name\":\"Exposure\",\"value\":"+Exposure+"},";
        config=config+ "{\"name\":\"Brightness\",\"value\":"+Brightness+"},";
        config=config+ "{\"name\":\"raw_Brightness\",\"value\":"+raw_Brightness+"}]}";
        
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);
        
//        CameraServer.getServer().getSource().setConfigJson(config);
        System.out.println(CameraServer.getServer().getSource().getConfigJson());

        cvSink = CameraServer.getVideo();

        outputStream = CameraServer.putVideo("Blur", 640, 480);
        Mat source = new Mat();

      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }

        if(SmartDashboard.getBoolean("updateCam", false)!=updateCamPrev){
          raw_Brightness=(int) SmartDashboard.getNumber("raw_Brightness", 1);
          Brightness=(int) SmartDashboard.getNumber("Brightness", 1);
          raw_Exposure=(int) SmartDashboard.getNumber("raw_Exposure", 1);
          Exposure=(int) SmartDashboard.getNumber("Exposure", 1);
          updateCamPrev=SmartDashboard.getBoolean("updateCam", false);
          config = "{\"properties\":[";
          config=config+ "{\"name\":\"raw_Exposure\",\"value\":"+raw_Exposure+"},";
          config=config+ "{\"name\":\"Exposure\",\"value\":"+Exposure+"},";
          config=config+ "{\"name\":\"Brightness\",\"value\":"+Brightness+"},";
          config=config+ "{\"name\":\"raw_Brightness\",\"value\":"+raw_Brightness+"}]}";          
          CameraServer.getServer().getSource().setConfigJson(config);          
        }

        
        bluePipe.process(source);            
          int numcontours=bluePipe.filterContoursOutput().size();
          if(numcontours>0){
            Imgproc.drawContours(source,
            bluePipe.filterContoursOutput(), -1, new Scalar(255,0,0), 2);                   
            outputStream.putFrame(source);        
            SmartDashboard.putNumber("Num Contours", numcontours);
            maxarea=0;
            area=0;
            maxareaContour=0;           
            i=0;
              while (i<numcontours){
                    area =Imgproc.contourArea(bluePipe.filterContoursOutput().get(i)); 
                    if (area >maxarea){
                        maxarea=area;
                        maxareaContour=i;
                    }
                i++;
              }
                        
            Moments M =Imgproc.moments(bluePipe.filterContoursOutput().get(maxareaContour)) ;
            centerX=(M.m10/M.m00);
            centerY=(M.m01/M.m00);
            distance=115*Math.exp(-centerY*0.00497);
            angle= 30*(centerX-320)/320;

            Rect rect = Imgproc.boundingRect(bluePipe.filterContoursOutput().get(maxareaContour)); 
            rectTLy= rect.tl().y;
            rectBRy= rect.br().y;

            rectTLx= rect.tl().x;
            rectBRx= rect.br().x;

            SmartDashboard.putNumber("ball centerX", centerX);
            SmartDashboard.putNumber("ball centerY", centerY);
            SmartDashboard.putNumber("ball distance", distance);
            SmartDashboard.putNumber("ball angle", angle);
            SmartDashboard.putNumber("rect tl x", rectTLx);
            SmartDashboard.putNumber("rect tl y", rectTLy);
            SmartDashboard.putNumber("rect br x", rectBRx);
            SmartDashboard.putNumber("rect br y", rectBRy);



          }
          else {
            outputStream.putFrame(source);
            centerX=-1;
            centerY=-1;       
          } 
      }
    }).start();
  }

  public double  getCenterX(){
    return centerX;
  }

  public double  getCenterY(){
    return centerY;
  }


  public double getDistance(){
    return distance;
  }

  public double getAngle(){
    return angle;
  }



}
