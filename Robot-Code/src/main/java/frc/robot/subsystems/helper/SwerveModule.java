package frc.robot.subsystems.helper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Interfaces.Motors.Motor;
import frc.robot.Interfaces.Motors.MotorWithEncoder;

public class SwerveModule  {
    
    private Motor driveMotor;
    private MotorWithEncoder steerMotor;

    private PIDController steerPID;

    public SwerveModule(Motor driveMotor, MotorWithEncoder steerMotor)
    {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        this.steerPID = new PIDController(.5, 0, 0);
        this.steerPID.enableContinuousInput(-Math.PI, Math.PI);
        this.reset();
    }

    public void setDesiredState(SwerveModuleState desiredState)
    {
        this.driveMotor.printToSmartDashboard();
        this.steerMotor.printToSmartDashboard();

        if(Math.abs(desiredState.speedMetersPerSecond) < .2)
        {
            //remove jitter?
            driveMotor.setSpeed(0);
            steerMotor.setSpeed(0);
            return;
        }
        
        desiredState.optimize(getAngle());

        driveMotor.setSpeed(desiredState.speedMetersPerSecond); // Not sure if this is correct // should prob be in voltage
        
        double turningSpeed = steerPID.calculate(this.getAngle().getRadians(), desiredState.angle.getRadians());
        SmartDashboard.putNumber(steerMotor.getName() + " error", steerPID.getError());
        SmartDashboard.putString(driveMotor.getName() + " and " + steerMotor.getName() + " Module", this.getAngle().getRadians() + " -> " + desiredState.angle.getRadians());
        turningSpeed = MathUtil.clamp(turningSpeed, -.25, .25); //.25 should be a constants
        steerMotor.setSpeed(turningSpeed);
        //steerMotor.setSpeed(turningSpeed);
        //SmartDashboard.putString(null, );
    }

    public Rotation2d getAngle()
    {
        // THIS IS BIG BAD
        // Creating a new Object every single time it is called
        return new Rotation2d(this.steerMotor.getRadian());
    }

    private void reset()
    {
        this.steerMotor.setPosition(0);
    }
}
