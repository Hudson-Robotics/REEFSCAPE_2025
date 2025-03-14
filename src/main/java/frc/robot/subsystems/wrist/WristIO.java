package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double velocity = 0.0;
    public double absoluteEncoderPosition = 0.0;

    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setVoltage(double speed) {}

  public default void resetRelativeEncoder() {}

  public default double getPosition() {
    return 0;
  }
}
