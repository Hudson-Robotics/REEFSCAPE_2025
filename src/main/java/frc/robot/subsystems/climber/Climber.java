package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private double targetSpeed = 0.0;

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void setSpeedRaw(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    io.setClimberVoltage(speed * Constants.ClimberConstants.maxClimberMotorVoltage);
  }

  public void setSpeed(double speed) {
    targetSpeed = speed;
  }

  public double getLeftClimberVelocity() {
    return inputs.leftClimberVelocityRadPerSec;
  }

  public double getRightClimberVelocity() {
    return inputs.rightClimberVelocityRadPerSec;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    setSpeedRaw(targetSpeed);
  }

  public void stop() {
    io.stop();
  }
}
