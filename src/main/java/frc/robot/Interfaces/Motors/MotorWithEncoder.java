package frc.robot.Interfaces.Motors;

public interface MotorWithEncoder extends Motor {
  double getPosition();

  double getAngle();

  double getRadian();

  double getSpeed();
}
