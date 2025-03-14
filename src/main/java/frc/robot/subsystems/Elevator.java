package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.Motors.Motor;
import frc.robot.Interfaces.interElevator;

public class Elevator extends SubsystemBase implements interElevator {

  final Motor eleMotor1;
  final Motor eleMotor2;
  private XboxController controller;

  public Elevator(XboxController controller, Motor eleMotor1, Motor eleMotor2) {
    controller = new XboxController(2);
    this.controller = controller;

    this.eleMotor1 = eleMotor1; // might be more useful to name this as left and right motors
    this.eleMotor2 = eleMotor2;
  }

  @Override
  public void periodic() {
    // Get the state of the RB button (boolean)
    boolean RBButtonPressed =
        controller.getRightBumper(); // Look into xbox controller documentation frc
    SmartDashboard.putBoolean("RB Button Pressed", RBButtonPressed);
    // going up+
    if (RBButtonPressed) {
      this.raise();
    }

    // to go down-
    boolean LBButtonPressed = controller.getLeftBumper();
    SmartDashboard.putBoolean("LB Button Pressed", LBButtonPressed);
    if (LBButtonPressed) {
      this.drop();
    }
  }

  // the elevators are mirrored so they might have to be inverted
  @Override
  public void raise() {
    this.setSpeed(.05);
  }

  @Override
  public void drop() {
    this.setSpeed(-.05);
  }

  @Override
  public void stop() {
    this.setSpeed(0);
  }

  @Override
  public void setSpeed(double speed) {
    this.eleMotor1.setSpeed(speed);
    this.eleMotor2.setSpeed(speed);
  }

  @Override
  public void setHeight(double height) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setHeight'");
  }
}
