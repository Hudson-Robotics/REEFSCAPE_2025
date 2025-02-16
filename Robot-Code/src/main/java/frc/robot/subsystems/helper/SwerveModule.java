package frc.robot.subsystems.helper;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule  {
    
    private SparkMax driveMotor;
    private RelativeEncoder driveEncoder;

    private SparkMax steerMotor;
    private RelativeEncoder steerEncoder;
    private PIDController steerPID;

    public SwerveModule(int driveMotorId, int steerMotorId)
    {
        this.driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        this.driveEncoder = driveMotor.getEncoder();

        this.steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);
        this.steerEncoder = steerMotor.getAlternateEncoder();
        this.steerPID = new PIDController(.5, 0, 0);
    }

    public void setDesiredState(SwerveModuleState desiredState)
    {
        //todo: replace optimize with the proper way of doing it.
        desiredState = SwerveModuleState.optimize(desiredState, this.getAngle());

        driveMotor.set(desiredState.speedMetersPerSecond); // Not sure if this is correct
        
        double turningSpeed = steerPID.calculate(getAngle().getRadians(), desiredState.angle.getRadians());
        MathUtil.clamp(turningSpeed, -.25, .25); //.25 should be a constants
        steerMotor.set(turningSpeed);
    }

    public Rotation2d getAngle()
    {
        // THIS IS BIG BAD
        // Creating a new Object every single time it is called
        return new Rotation2d(steerEncoder.getPosition() * 2 * Math.PI);
    }
}
