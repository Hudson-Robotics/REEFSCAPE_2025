package frc.robot.subsystems.helper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class SwerveModule  {
    
    private MotorWithEncoder driveMotor;
    private MotorWithEncoder steerMotor;

    private PIDController steerPID;

    public SwerveModule(MotorWithEncoder driveMotor, MotorWithEncoder steerMotor)
    {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        this.steerPID = new PIDController(.1, 0, 0);
        this.steerPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setDesiredState(SwerveModuleState desiredState)
    {
        //todo: replace optimize with the proper way of doing it.
        desiredState.optimize(getAngle());

        driveMotor.setSpeed(desiredState.speedMetersPerSecond); // Not sure if this is correct // should prob be in voltage
        
        SmartDashboard.putNumber("Measurement", this.getAngle().getRadians());
        SmartDashboard.putNumber("Set", desiredState.angle.getRadians());
        double turningSpeed = steerPID.calculate(this.getAngle().getRadians(), desiredState.angle.getRadians());
        MathUtil.clamp(turningSpeed, -.25, .25); //.25 should be a constants
        steerMotor.setSpeed(turningSpeed);
        //SmartDashboard.putString(null, );

        this.driveMotor.printToSmartDashboard();
        this.steerMotor.printToSmartDashboard();
    }

    public Rotation2d getAngle()
    {
        // THIS IS BIG BAD
        // Creating a new Object every single time it is called
        return new Rotation2d(this.steerMotor.getAngle());
    }
}
