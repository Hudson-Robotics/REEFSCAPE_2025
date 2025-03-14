// no sue plz :D

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.DriveTrain;
import frc.robot.subsystems.helper.SwerveModule;

public class SwerveDrive extends SubsystemBase implements DriveTrain {
  // xbox
  private XboxController controller;
  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

  Translation2d frontLeftLocation =
      new Translation2d(-.381, .381); // .381 is half of .762 m which is 30 inches in freedom units
  Translation2d frontRightsLocation =
      new Translation2d(.381, .381); // .381 is half of .762 m which is 30 inches in freedom unitS
  Translation2d backLeftLocation =
      new Translation2d(-.381, -.381); // .381 is half of .762 m which is 30 inches in freedom units
  Translation2d backRightLocation =
      new Translation2d(.381, -.381); // .381 is half of .762 m which is 30 inches in freedom units

  SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightsLocation, backLeftLocation, backRightLocation);

  public SwerveDrive(SwerveModule flm, SwerveModule frm, SwerveModule blm, SwerveModule brm) {
    controller = new XboxController(1);

    this.frontLeftModule = flm;
    this.frontRightModule = frm;
    this.backLeftModule = blm;
    this.backRightModule = brm;
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    double speedX = this.controller.getLeftX();
    double speedY = this.controller.getLeftY();
    double rotate = this.controller.getRightY();
    // maybe add slewrates....

    var SwerveModuleState =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rotate, new Rotation2d(0)));
    frontLeftModule.setDesiredState(SwerveModuleState[0]);
    frontRightModule.setDesiredState(SwerveModuleState[1]);
    backLeftModule.setDesiredState(SwerveModuleState[2]);
    backRightModule.setDesiredState(SwerveModuleState[3]);

    SmartDashboard.putString("Front Left", SwerveModuleState[0].toString());
    SmartDashboard.putString("Front Right", SwerveModuleState[1].toString());
    SmartDashboard.putString("Back Left", SwerveModuleState[2].toString());
    SmartDashboard.putString("Back Right", SwerveModuleState[3].toString());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void drive(final double xspeed, final double yspeed, final double rotation) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'drive go'");
  }
}

public class DriveSubsystem {
  private static final int LEFT_FRONT_ID = 1;
  private static final int RIGHT_FRONT_ID = 2;
  private static final int LEFT_BACK_ID = 3;
  private static final int RIGHT_BACK_ID = 4;
  private static final int REAR_RIGHT_BACK_ID = 5;
  private static final int REAR_LEFT_BACK_ID = 6;
  private static final int REAR_RIGHT_FRONT_ID = 7;
  private static final int REAR_LEFT_FRONT_ID = 8;
  private static final int FRONT_LEFT_ENCODER_ID = 9;
  private static final int FRONT_RIGHT_ENCODER_ID = 10;
  private static final int BACK_LEFT_ENCODER_ID = 11;
  private static final int BACK_RIGHT_ENCODER_ID = 12;

  // Define Spark MAX motor controllers
  private final CANSparkMax leftFront = new CANSparkMAX(LEFT_FRONT_ID, Motortype.kBrushless);
  private final CANSparkMax rightFront = new CANSparkMAX(RIGHT_FRONT_ID, Motortype.kBrushless);
  private final CANSparkMax leftBack = new CANSparkMAX(LEFT_BACK_ID, Motortype.kBrushless);
  private final CANSparkMax rightBack = new CANSparkMAX(RIGHT_BACK_ID, Motortype.kBrushless);
  private final CANSparkMax rearRightBack =
      new CANSparkMAX(REAR_RIGHT_BACK_ID, Motortype.kBrushless);
  private final CANSparkMax rearLeftBack = new CANSparkMAX(REAR_LEFT_BACK_ID, Motortype.kBrushless);
  private final CANSparkMax rearRightFront =
      new CANSparkMAX(REAR_RIGHT_FRONT_ID, Motortype.kBrushless);
  private final CANSparkMax rearLeftFront =
      new CANSparkMAX(REAR_LEFT_FRONT_ID, Motortype.kBrushless);

  // Define External CANcoders
  private final CANcoder leftEncoder = new CANCoder(FRONT_LEFT_ENCODER_ID);
  private final CANCoder rightEncoder = new CANCoder(FRONT_RIGHT_ENCODER_ID);
  private final CANCoder backEncoder = new CANCoder(BACK_LEFT_ENCODER_ID);
  private final CANCoder frontEncoder = new CANCoder(BACK_RIGHT_ENCODER_ID);

  public DriveSubsystem() {
    // Restore factory defaults
    leftFront.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();
    rearRightBack.restoreFactoryDefaults();
    rearLeftBack.restoreFactoryDefaults();
    rearRightFront.restoreFactoryDefaults();
    rearLeftFront.restoreFactoryDefaults();

    // Configure follower motors
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    // Set motor inversions (adjust if needed)
    leftFront.setInverted(false);
    rightFront.setInverted(true);

    // Apply CANcoder configurations (if needed)
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    leftEncoder.getConfigurator().apply(cancoderConfig);
    rightEncoder.getConfigurator().apply(cancoderConfig);
    backEncoder.getConfigurator().apply(cancoderConfig);
    frontEncoder.getConfigurator().apply(cancoderConfig);
  }

  // Get CANcoder positions (rotations)
  public double getLeftPosition() {
    return leftEncoder.getPosition().getValue();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition().getValue();
  }

  // need to change
  public double getBackPosition() {
    return backEncoder.getPosition().getValue();
  }

  // need to change
  public double getFrontPosition() {
    return frontEncoder.getPosition().getValue();
  }

  // Get velocity (RPM)
  public double getLeftVelocity() {
    return leftEncoder.getVelocity().getValue();
  }

  public double getRightVelocity() {
    return rightEncoder.getVelocity().getValue();
  }

  // Reset encoders
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    backEncoder.setPosition(0);
    frontEncoder.setPosition(0);
  }

  // Tank drive method (for teleop)
  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftFront.set(leftSpeed);
    rightFront.set(rightSpeed);
  }
}

public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;
  private final CANcoder cancoder;

  private final double angleOffset;

  public SwerveModule(int driveMotorID, int turnMotorID, int cancoderID, double offset) {
    driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    cancoder = new CANcoder(cancoderID);
    angleOffset = offset;

    // Reset motor to factory defaults
    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();
  }

  // Get absolute angle (in degrees)
  public double getAngle() {
    return (cancoder.getAbsolutePosition().getValue() * 360.0) - angleOffset;
  }

  // Set drive speed
  public void setDriveSpeed(double speed) {
    driveMotor.set(speed);
  }

  // Set turn motor power (for steering)
  public void setTurnPower(double power) {
    turnMotor.set(power);
  }
}

public class SwerveDrive {
  public final SwerveModule frontLeft;
  public final SwerveModule frontRight;
  public final SwerveModule backLeft;
  public final SwerveModule backRight;

  public SwerveDrive() {
    frontLeft = new SwerveModule(1, 2, 3, 0.0);
    frontRight = new SwerveModule(4, 5, 6, 0.0);
    backLeft = new SwerveModule(7, 8, 9, 0.0);
    backRight = new SwerveModule(10, 11, 12, 0.0);
  }

  public void drive(double speed, double rotation) {
    frontLeft.setDriveSpeed(speed);
    frontRight.setDriveSpeed(speed);
    backLeft.setDriveSpeed(speed);
    backRight.setDriveSpeed(speed);

    // Example: Turn each module based on input (simplified)
    frontLeft.setTurnPower(rotation);
    frontRight.setTurnPower(rotation);
    backLeft.setTurnPower(rotation);
    backRight.setTurnPower(rotation);
  }
}
