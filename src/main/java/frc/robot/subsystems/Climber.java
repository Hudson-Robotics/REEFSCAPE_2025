package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.Climb;
import frc.robot.Interfaces.Motors.Motor;

public class Climber extends SubsystemBase implements Climb {
  final Motor upMotor1;
  final Motor UpMotor2;

  private XboxController controller;

  public Climber(Motor upMotor1, Motor upMotor2) {
    controller = new XboxController(2);
    this.upMotor1 = upMotor1;
    this.UpMotor2 = upMotor2;
  }

  @Override
  public void periodic() {
    // Get the state of the A button (boolean)
    boolean xButtonPressed = controller.getAButton();
    SmartDashboard.putBoolean("X Button Pressed", xButtonPressed);
    if (xButtonPressed) {
      this.climb(1);
    }
    // Get the state of the A button (boolean)
    boolean xButtonPressed = controller.getAButton();
    SmartDashboard.putBoolean("X Button Pressed", xButtonPressed);
    SmartDashboard.putNumber("upMotor1", .01);
    upMotor1.set(.01);
    SmartDashboard.putNumber("upMotor2", .01);
    UpMotor2.set(.01);
  }

  @Override
  public void climb(double speed) {
    this.upMotor1.setSpeed(speed);
    this.UpMotor2.setSpeed(speed);
  }
}
