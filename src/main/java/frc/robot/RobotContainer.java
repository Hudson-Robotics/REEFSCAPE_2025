// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SparkMaxIDs;
import frc.robot.Interfaces.Motors.MotorWithEncoder;
import frc.robot.commands.AMoveEnd;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.helper.SparkMaxBrushlessEncoderMotor;
import frc.robot.subsystems.helper.SwerveModule;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private final Wrist wrist;
  private final Drive drive;
  private final Intake intake;
  private final Climber climber;
  private final Elevator elevator;
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Led leds = new Led();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>();

  // The driver's and operators controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Configure the button bindings
    configureButtonBindings();

    autoChooser.setDefaultOption("Cross Auto Line Only", new AMoveEnd(m_robotDrive));
    autoChooser.addOption(
        "Do Nothing", new RunCommand(() -> m_robotDrive.drive(0.0, 0.0, 0.0, true), m_robotDrive));

    SmartDashboard.putData("Auto Choices", autoChooser);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));
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
    driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  private void configureButtonBindings() {
    new JoystickButton(driverController, Button.kR1.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    // Resets direction to 0 degrees
    new JoystickButton(driverController, Button.kL1.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.get();
  }

  private SwerveDrive createSwerveDrive() {
    MotorWithEncoder frontLeftDrive =
        new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_LEFT_DRIVE, "FrontLeftDrive", false, 1);
    MotorWithEncoder frontLeftSteer =
        new SparkMaxBrushlessEncoderMotor(
            SparkMaxIDs.FRONT_LEFT_STEER, "FrontLeftSteer", true, 360);
    SwerveModule frontLeft = new SwerveModule(frontLeftDrive, frontLeftSteer);

    MotorWithEncoder frontRightDrive =
        new SparkMaxBrushlessEncoderMotor(
            SparkMaxIDs.FRONT_RIGHT_DRIVE, "frontRightDrive", false, 1);
    MotorWithEncoder frontRightSteer =
        new SparkMaxBrushlessEncoderMotor(
            SparkMaxIDs.FRONT_RIGHT_STEER, "frontRightSteer", true, 360);
    SwerveModule frontRight = new SwerveModule(frontRightDrive, frontRightSteer);

    MotorWithEncoder backLeftDrive =
        new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.BACK_LEFT_DRIVE, "backLeftDrive", false, 1);
    MotorWithEncoder backLeftSteer =
        new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.BACK_LEFT_STEER, "backLeftSteer", true, 360);
    SwerveModule backLeft = new SwerveModule(backLeftDrive, backLeftSteer);

    MotorWithEncoder backRightDrive =
        new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.BACK_RIGHT_DRIVE, "backRightDrive", false, 1);
    MotorWithEncoder backRightSteer =
        new SparkMaxBrushlessEncoderMotor(
            SparkMaxIDs.BACK_RIGHT_STEER, "backRightSteer", true, 360);
    SwerveModule backRight = new SwerveModule(backRightDrive, backRightSteer);

    return new SwerveDrive(frontLeft, frontRight, backLeft, backRight);
  }
}
