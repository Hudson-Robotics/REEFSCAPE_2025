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

// import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PS4Controller.Button; //change to Dualsense or add dual sense
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MainCommands;
import frc.robot.commands.AMoveEnd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import java.util.List;
// look at vector8177_2025robot & team9181's 2025 robot
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
  private final Wrist wrist;
  private final Drive drive;
  private final Intake intake;
  private final Climber climber;
  private final Elevator elevator;
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  //private final SecurityCam camera;
  private final SwerveDrive drivetrain = new SwerveDrive();
  private final Led leds = new Led();
 
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // The driver's and operators controller
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // Configure your button bindings here
  //private final CommandXboxController m_driverController = 
      //new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    // Resets direction to 0 degrees
    new JoystickButton(m_drivercontroller, Button.kL1.values()
        .whileTrue(new RunCommand(
          () -> m_robotDrive.zeroHeading(),
          m_robotDrive));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Configure the button bindings
    configureButtonBindings();
    //camera = new SecurityCam();
    
    autoChooser.setDefaultOption("Cross Auto Line Only", new AMoveEnd(m_robotDrive));
    autoChooser.addOption("Do Nothing",
      new RunCommand(
        ()-> m_robotDrive.drive(0.0,0.0,0.0,true), m_robotDrive)
    );

    SmartDashboard.putData("Auto Choices", autoChooser);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
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
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.get();
    return Autos.exampleAuto(m_exampleSubsystem);
     return autoChooser.getSelected();
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    */
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
