package frc.robot.subsystems;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SecurityCam extends SubsystemBase {
    private final int VOLTAGE_MAX = 5;
    private final int BRIGHTNESS_MAX = 255;

    private final int WIDTH = 976;
    private final int HEIGHT = 582;
    
    private final int cameraFeedAnalogInputId = 0;
    private final AnalogInput cameraFeed;

    private final CvSource outputStream;

    private final int[] frameBuffer;
    private Mat frame;

    public SecurityCam() {
        this.cameraFeed = new AnalogInput(cameraFeedAnalogInputId);
        this.frameBuffer = new int[this.WIDTH * this.HEIGHT];

        outputStream = CameraServer.putVideo("Security Cam", WIDTH, HEIGHT);
        frame = new Mat(HEIGHT, WIDTH, CvType.CV_8UC3); // Gray image

        frame.setTo(new Scalar(120, 120, 120));
        outputStream.putFrame(frame);
    }

    @Override
    public void periodic() {
        // TODO: Make this function render every other line and then alternate each frame to simulate TVL
        // Periodic runs at 50 Hz or every 20 ms
        this.captureFrame();
        this.renderFrame();
        this.displayFrame();
    }

    // This is reading from the Analog Input and storing it in an array
    // This is very slow becuase it waits for every pixel to be filled in
    // This might not even be the right way to store the data because its a TVL
    private void captureFrame() {
        for(int i = 0; i < this.WIDTH * this.HEIGHT; i++)
        {
            this.frameBuffer[i] = (int) (this.cameraFeed.getVoltage() * this.BRIGHTNESS_MAX / this.VOLTAGE_MAX);
        }
    }

    // Converts the frameBuffer (1 D) to a Frame (3 D (Width x Height x Pixel))
    private void renderFrame() {
        for (int row = 0; row < this.HEIGHT; row++) {
            for (int col = 0; col < this.WIDTH; col++) {
                int index = row * this.WIDTH + col;
                int value = this.frameBuffer[index];
                this.frame.put(row, col, value, value, value); // three value for RGB, and if R G B are the same value its gray scale
            }
        }
    }

    // Writes out the Frame to the Driver Station
    private void displayFrame() {
        this.outputStream.putFrame(frame);
    }

}
