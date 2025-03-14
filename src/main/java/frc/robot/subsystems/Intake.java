package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.Motors.Motor;
import frc.robot.Interfaces.interIntake;

public class Intake extends SubsystemBase implements interIntake {
  final Motor intakeMotor1;
  final Motor intakeMotor2;

  private XboxController controller;

  public Intake(Motor intakeMotor1, Motor intakeMotor2) {
    controller = new XboxController(2);
    this.intakeMotor1 = intakeMotor1;
    this.intakeMotor2 = intakeMotor2;
  }

  @Override
  public void periodic() {
    // Get the state of the A button (boolean)
    boolean xButtonPressed = controller.getAButton();
    SmartDashboard.putBoolean("X Button Pressed", xButtonPressed);
    if (xButtonPressed) {
      this.intake();
    }

    boolean bButtonPressed = controller.getBButton();
    SmartDashboard.putBoolean("B Button Pressed", bButtonPressed);
    if (bButtonPressed) {
      this.outtake();
    }
  }

  @Override
  public void intake() {
    this.setSpeed(.05);
  }

  @Override
  public void outtake() {
    this.setSpeed(-.05);
  }

  @Override
  public void setSpeed(double speed) {
    this.intakeMotor1.setSpeed(speed);
    this.intakeMotor2.setSpeed(speed);
  }

  @Override
  public boolean hasCoral() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'hasCoral'");
  }
}
