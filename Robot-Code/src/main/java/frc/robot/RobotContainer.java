// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SparkMaxIDs;
import frc.robot.Interfaces.Motors.MotorWithEncoder;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.helper.SparkMaxBrushlessEncoderMotor;
import frc.robot.subsystems.helper.SwerveModule;
import frc.robot.subsystems.Led;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
 private final SwerveDrive drivetrain = this.createSwerveDrive();
 private final Led leds = new Led();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //camera = new SecurityCam();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  private SwerveDrive createSwerveDrive() {
    MotorWithEncoder frontLeftDrive = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_LEFT_DRIVE, "FrontLeftDrive", false, 1);
    MotorWithEncoder frontLeftSteer = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_LEFT_STEER, "FrontLeftSteer", true, 360);
    SwerveModule frontLeft = new SwerveModule(frontLeftDrive, frontLeftSteer);

    MotorWithEncoder frontRightDrive = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_RIGHT_DRIVE, "frontRightDrive", false, 1);
    MotorWithEncoder frontRightSteer = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_RIGHT_STEER, "frontRightSteer", true, 360);
    SwerveModule frontRight = new SwerveModule(frontRightDrive, frontRightSteer);

    MotorWithEncoder backLeftDrive = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.BACK_LEFT_DRIVE, "backLeftDrive", false, 1);
    MotorWithEncoder backLeftSteer = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.BACK_LEFT_STEER, "backLeftSteer", true, 360);
    SwerveModule backLeft = new SwerveModule(backLeftDrive, backLeftSteer);

    MotorWithEncoder backRightDrive = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_LEFT_DRIVE, "backRightDrive", false, 1);
    MotorWithEncoder backRightSteer = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_LEFT_STEER, "backRightSteer", true, 360);
    SwerveModule backRight = new SwerveModule(backRightDrive, backRightSteer);

    return new SwerveDrive(frontLeft, frontRight, backLeft, backRight);
  }
}
