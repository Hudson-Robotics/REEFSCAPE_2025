package frc.robot.subsystems.helper;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Motors.Motor;

public class SparkMaxBrushlessMotor implements Motor {
  protected SparkMax motor;
  private final String TYPE = "SPARK MAX";
  private int id;
  private String name;

  public double limit = .1;

  public SparkMaxBrushlessMotor(int canBusId, String motorName) {
    this.name = motorName;
    this.id = canBusId;

    this.motor = new SparkMax(id, MotorType.kBrushless);
  }

  @Override
  public void setSpeed(double speed) {
    double clampedValue = MathUtil.clamp(speed, -this.limit, this.limit);
    this.motor.set(clampedValue);
  }

  @Override
  public double getSpeed() {
    return this.motor.get();
  }

  @Override
  public String getName() {
    return String.format("%s (%s %d)", this.name, this.TYPE, this.id);
  }

  @Override
  public void printToSmartDashboard() {
    SmartDashboard.putNumber(this.getName(), this.getSpeed());
  }
}
