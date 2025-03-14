package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private double targetPosition;
  private final PIDController pidController;
  private ArmFeedforward feedForward;

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {

    this.io = io;
    pidController = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    pidController.setTolerance(.25);
    io.resetRelativeEncoder();

    feedForward =
        new ArmFeedforward(
            WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    Logger.recordOutput("Wrist Position", io.getPosition());

    double pidMotorSpeed =
        pidController.calculate(io.getPosition(), targetPosition)
            + feedForward.calculate(targetPosition, 0);
    setMotor(
        MathUtil.clamp(
            (pidMotorSpeed), -WristConstants.MAX_WRIST_VOLTAGE, WristConstants.MAX_WRIST_VOLTAGE));
  }

  public void setMotor(double voltage) {
    io.setVoltage(voltage);
  }

  public void setPosition(double position) {
    // Logger.getInstance().recordOutput("WristTargetPosition", position);
    Logger.recordOutput("Wrist Target Position", position);
    targetPosition = position;
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void resetRelativeEncoder() {
    io.resetRelativeEncoder();
  }

  public void setWristSetpoint(double offset) {
    io.setPosition(io.getPosition() + offset);
  }
}
