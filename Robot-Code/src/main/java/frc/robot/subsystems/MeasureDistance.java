package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.Climb;

import frc.robot.Interfaces.Motors.Motor;
import frc.robot.subsystems.helper.LimelightHelpers;

public class MeasureDistance extends SubsystemBase{
 final LimelightHelpers llh = new LimelightHelpers();
 final AnalogInput ai = new AnalogInput(0);

 public MeasureDistance() { 
}

    @Override
     public void periodic() {
        int id = (int)LimelightHelpers.getFiducialID("limelight");
        Logger.recordOutput(id + ": x", LimelightHelpers.getTX("limelight"));
        Logger.recordOutput(id + ": y", LimelightHelpers.getTY("limelight"));
        Logger.recordOutput(id + ": a", LimelightHelpers.getTA("limelight"));

        Logger.recordOutput("Ultrasonic", ai.getValue());
     }
}