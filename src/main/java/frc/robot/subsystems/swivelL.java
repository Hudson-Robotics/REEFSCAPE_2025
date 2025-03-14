package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.Motors.Motor;
import frc.robot.Interfaces.swivel;

public class swivelL extends SubsystemBase implements swivel {
  final Motor swivelMotor;
  private XboxController controller;

  public swivelL(XboxController controller, Motor swivelMotor) {
    this.controller = controller;
    this.swivelMotor = swivelMotor;
  }

  @Override
  public void periodic() {
    double speedX = this.controller.getLeftX();
    this.swivelMotor.setSpeed(speedX);
  }

  @Override
  public void up() {
    this.swivelMotor.setSpeed(.05);
  }

  @Override
  public void down() {
    this.swivelMotor.setSpeed(-.05);
  }

  @Override
  public void stop() {
    this.swivelMotor.setSpeed(0);
  }
}
