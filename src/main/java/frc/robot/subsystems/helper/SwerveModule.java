package frc.robot.subsystems.helper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class SwerveModule {

  private MotorWithEncoder driveMotor;
  private MotorWithEncoder steerMotor;

  private PIDController steerPID;

  public SwerveModule(MotorWithEncoder driveMotor, MotorWithEncoder steerMotor) {
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;

    this.steerPID = new PIDController(.5, 0, 0);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // todo: replace optimize with the proper way of doing it.
    desiredState.optimize(getAngle());

    driveMotor.setSpeed(
        desiredState
            .speedMetersPerSecond); // Not sure if this is correct // should prob be in voltage

    double turningSpeed =
        steerPID.calculate(this.getAngle().getRadians(), desiredState.angle.getRadians());
    MathUtil.clamp(turningSpeed, -.25, .25); // .25 should be a constants
    steerMotor.setSpeed(turningSpeed);
  }

  public Rotation2d getAngle() {
    // THIS IS BIG BAD
    // Creating a new Object every single time it is called
    return new Rotation2d(this.steerMotor.getAngle());
  }
}
