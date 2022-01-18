package frc.robot.Sensors;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;

public class advancedCam {
    static CvSink cvSink;
    static CvSource outputStream;
 
public advancedCam(){
    getVideo();
}

public static void getVideo() { 
// this is an inline thread that just runs repeatedly
// you could add a reference to a static variable within the thread that
// will turn on and off the image processing, change pipeline, etc.            
    new Thread(() -> {
        String config = "{\"properties\":[{\"name\":\"raw_Exposure\",\"value\":-12},";
        config=config+ "{\"name\":\"Exposure\",\"value\":-12},";
        config=config+ "{\"name\":\"Brightness\",\"value\":1}]}";
//        config=config+ "{\"name\":\"raw_Brightness\",\"value\":1}]}";
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);
        
        CameraServer.getServer().getSource().setConfigJson(config);
        System.out.println(CameraServer.getServer().getSource().getConfigJson());

        cvSink = CameraServer.getVideo();

        outputStream = CameraServer.putVideo("Blur", 640, 480);
        Mat source = new Mat();
        Mat output = new Mat();
      while(!Thread.interrupted()) {
        if (cvSink.grabFrame(source) == 0) {
          continue;
        }
// Here is where you could do image processing
// like making calls to a grip pipeline, resizing, changing color, etc.
Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);

    outputStream.putFrame(source); 
      }
    }).start();
  }
}
