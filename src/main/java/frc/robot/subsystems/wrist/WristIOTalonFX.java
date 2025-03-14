package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.WristConstants;

public class WristIOTalonFX implements WristIO {
  private final TalonFX wristMotor;
  private final TalonFXConfiguration configuration;

  public WristIOTalonFX() {
    wristMotor = new TalonFX(WristConstants.WRIST_MOTOR_ID);
    configuration = new TalonFXConfiguration();

    wristMotor.getConfigurator().apply(configuration, .05);

    wristMotor.setPosition(0);

    wristMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {}

  @Override
  public void resetRelativeEncoder() {
    wristMotor.setPosition(0);
  }

  @Override
  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    wristMotor.setPosition(position);
  }
}
