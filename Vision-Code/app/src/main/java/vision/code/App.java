/*
 * This source file was generated by the Gradle 'init' task
 */
package vision.code;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

public class App {

  public static void main(String[] args) {
    UsbCamera camera = new UsbCamera("TEST", 0);
    System.out.println(camera.getActualFPS());
    camera.close();
  }
}

