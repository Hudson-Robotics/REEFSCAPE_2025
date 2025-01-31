package frc.robot.HelperClasses;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class NeoSwerve {
    private String Name;

    private SparkMax turningMotor;
    private RelativeEncoder altTurningEncoder;
    private PIDController turningPID;
    private double kP = 0.5;
    private double kI = 0;
    private double kD = 0;

    private SparkMax driveMotor;
    private RelativeEncoder driveEncoder;

    //private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;

    public NeoSwerve(int turningMotorCanbusAddress, int driveMotorCanbusAddress, String Name) {
        turningMotor = new SparkMax(turningMotorCanbusAddress, MotorType.kBrushless);
        altTurningEncoder = turningMotor.getAlternateEncoder(); //this required pin count and type we will see if it works

        driveMotor = new SparkMax(driveMotorCanbusAddress, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();

        this.Name = Name;

        turningPID = new PIDController(kP, kI, kD);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private Rotation2d getAngle() {
        return new Rotation2d(altTurningEncoder.getPosition() * 2 * Math.PI);
    }

    private double getSpeed() {
        double circumference = 2 * Math.PI * .0508; //Constants.WHEEL_RADIUS_IN_METERS;
        return driveEncoder.getVelocity() / (circumference / 60);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.1) {
            stop();
            return;
        }
        SwerveModuleState optomized = SwerveModuleState.optimize(desiredState, getAngle());

        driveMotor.set(optomized.speedMetersPerSecond);

        double turningSpeed = turningPID.calculate(getAngle().getRadians(), optomized.angle.getRadians());
        MathUtil.clamp(turningSpeed, -0.25, 0.25);
        turningMotor.set(turningSpeed);

        SmartDashboard.putNumber(Name + " Optomized Wheel Speed", optomized.speedMetersPerSecond);
        SmartDashboard.putNumber(Name + " Optomized Wheel Angle", optomized.angle.getRadians());
    }

    public void stop() {
        turningMotor.set(0);
        driveMotor.set(0);
    }

    public void updateOdometry() {
        SmartDashboard.putNumber(Name + " Output", turningMotor.get());
        SmartDashboard.putNumber(Name + " Postion Error", turningPID.getPositionError());

        SmartDashboard.putNumber(Name + " Alternate Encoder", altTurningEncoder.getPosition() * Math.PI * 2);
    }

    public void Reset() {
        System.out.println("Initial " + Name + " Encoder @ " + altTurningEncoder.getPosition());
        altTurningEncoder.setPosition(0);
        driveEncoder.setPosition(0);
        System.out.println("Resetting " + Name + " Encoder @ " + altTurningEncoder.getPosition());
    }
}