package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final CANSparkMax elevatorMotor;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pidController;

  public ElevatorIOSparkMax(int motorID) {
    elevatorMotor = new CANSparkMax(motorID, MotorType.kBrushless);
    encoder = elevatorMotor.getEncoder();
    pidController = elevatorMotor.getPIDController();

    // Restore factory defaults to ensure the motor controller is in a known state
    elevatorMotor.restoreFactoryDefaults();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.motorCurrent = elevatorMotor.getOutputCurrent();
    inputs.motorVoltage = elevatorMotor.getBusVoltage();
    inputs.motorSpeed = encoder.getVelocity();
    inputs.motorPosition = encoder.getPosition();
  }

  @Override
  public void setElevatorVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void setPosition(double position) {
    pidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void resetPosition() {
    encoder.setPosition(0);
  }

  @Override
  public void stop() {
    elevatorMotor.stopMotor();
  }
}
