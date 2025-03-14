package frc.robot.subsystems.helper;

import com.revrobotics.RelativeEncoder;
import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class SparkMaxBrushlessEncoderMotor extends SparkMaxBrushlessMotor
    implements MotorWithEncoder {
  private RelativeEncoder encoderMotor;
  private double conversionFactor;

  public SparkMaxBrushlessEncoderMotor(
      int canBusId, String motorName, boolean useAlternate, double conversionFactor) {
    super(canBusId, motorName);
    this.encoderMotor = useAlternate ? super.motor.getAlternateEncoder() : super.motor.getEncoder();
    this.conversionFactor = conversionFactor;
  }

  @Override
  public double getPosition() {
    return this.encoderMotor.getPosition() * this.conversionFactor;
  }

  @Override
  public double getAngle() {
    // Get the position in rotations
    double positionInRotations = this.encoderMotor.getPosition();

    // Convert the position to degrees
    double positionInDegrees = positionInRotations * 360.0;

    // Handle wrap-around to get the absolute angle in the range of -180 to 180 degrees
    double absoluteAngle = positionInDegrees % 360.0;

    if (absoluteAngle > 180.0) {
      absoluteAngle -= 360.0;
    } else if (absoluteAngle < -180.0) {
      absoluteAngle += 360.0;
    }

    return absoluteAngle;
  }

  @Override
  public double getRadian() {
    return (this.getAngle() * Math.PI) / 180;
  }
}
