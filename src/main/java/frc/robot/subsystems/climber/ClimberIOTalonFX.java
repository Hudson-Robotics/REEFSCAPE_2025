package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX leftClimberMotor;
  private final TalonFX rightClimberMotor;
  private final TalonFXConfiguration configuration;

  public ClimberIOTalonFX() {
    leftClimberMotor = new TalonFX(Constants.ClimberConstants.LEFT_MOTOR_ID); // Change later
    rightClimberMotor = new TalonFX(Constants.ClimberConstants.RIGHT_MOTOR_ID);

    configuration = new TalonFXConfiguration();
    leftClimberMotor.getConfigurator().apply(configuration, .05);
    rightClimberMotor.getConfigurator().apply(configuration, .05);

    rightClimberMotor.setControl(new Follower(Constants.ClimberConstants.LEFT_MOTOR_ID, true));

    leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftClimberAppliedVolts =
        leftClimberMotor.getDutyCycle().getValueAsDouble()
            * leftClimberMotor.getSupplyVoltage().getValueAsDouble();
    inputs.leftClimberVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            leftClimberMotor.getVelocity().getValueAsDouble());
    // inputs.leftClimberCurrentAmps = new double[] {leftClimberMotor.getOutputCurrent()};
    inputs.leftClimberEncoderPosition = leftClimberMotor.getPosition().getValueAsDouble();

    inputs.rightClimberAppliedVolts =
        rightClimberMotor.getDutyCycle().getValueAsDouble()
            * rightClimberMotor.getSupplyVoltage().getValueAsDouble();
    inputs.rightClimberVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            rightClimberMotor.getVelocity().getValueAsDouble());
    // inputs.rightClimberCurrentAmps = new double[] {rightClimberMotor.getOutputCurrent()};
    inputs.rightClimberEncoderPosition = rightClimberMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setClimberVoltage(double volts) {
    leftClimberMotor.setVoltage(volts);
    rightClimberMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    leftClimberMotor.setVoltage(0);
    rightClimberMotor.setVoltage(0);
  }
}
