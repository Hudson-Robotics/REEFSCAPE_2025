// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SparkMaxIDs;
import frc.robot.Interfaces.Motors.Motor;
import frc.robot.Interfaces.Motors.MotorWithEncoder;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Climb.ClampCage;
import frc.robot.commands.Climb.ClimbCage;
import frc.robot.commands.Climb.UnclampCage;
import frc.robot.commands.Elevator.RaiseElevator;
import frc.robot.commands.Intake.IntakeCoral;
import frc.robot.commands.Swivel.LowerToL1;
import frc.robot.commands.Swivel.SwivelJoy;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swivelL;
import frc.robot.subsystems.helper.SparkMaxBrushlessEncoderMotor;
import frc.robot.subsystems.helper.SparkMaxBrushlessMotor;
import frc.robot.subsystems.helper.SwerveModule;
import frc.robot.subsystems.helper.TalonFXMotor;
import frc.robot.subsystems.helper.TalonFXMotorWithEncoder;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Intake;

import java.io.File;
import java.util.Optional;

import javax.naming.InitialContext;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
//Driver 0 = Drive
//Driver 1 = Manipulators
final CommandXboxController driverXbox = new CommandXboxController(0);
final CommandXboxController manipulatorXbox = new CommandXboxController(1);

private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo")); //instead of reading file might be better to create it via code
/**
 * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
 */
SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                              () -> driverXbox.getLeftY() * -1,
                                                              () -> driverXbox.getLeftX() * -1)
                                                          .withControllerRotationAxis(()-> driverXbox.getRightX() * -1.0)
                                                          .deadband(OperatorConstants.DEADBAND)
                                                          .scaleTranslation(0.8)
                                                          .allianceRelativeControl(true);
  
  /**
 * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
 */                                                        
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);
                  
  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

 private final swivelL swivel = new swivelL(new TalonFXMotorWithEncoder(12, "Swivel")); // need to move canBusId to constants and remove the xboxController from the swivel class
 private final Led leds = new Led();
 private final Intake intake = createIntake();
 private final Elevator elevator = createElevator();
 private final Climber climber = createClimb();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("LowerArm", new LowerToL1(swivel));
    NamedCommands.registerCommand("Shoot", getAutonomousCommand());
    // Configure the trigger bindings
    configureBindings();
    //camera = new SecurityCam();
    this.mapControllers();
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
Command driveFieldOrientedDirectAngle      = driveBase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = driveBase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = driveBase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = driveBase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = driveBase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = driveBase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      driveBase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      driveBase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //driveBase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> driveBase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(driveBase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driverXbox.b().whileTrue(
//          driveBase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      driveBase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(driveBase::lock, driveBase).repeatedly());
      driverXbox.y().whileTrue(driveBase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(driveBase::zeroGyro)));
      driverXbox.back().whileTrue(driveBase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(driveBase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(driveBase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(driveBase::lock, driveBase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
  }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String filePath = getAllianceLetter() + " " + getGameData();
    if(filePath.length() < 3)
    {
      return driveBase.driveCommand(() -> -1, () -> 0, () -> 0);
    }
    return driveBase.getAutonomousCommand(filePath); // Add a way to get Alliance Color + Driver Input
  }

  private String getAllianceLetter() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        return "R";
      }
     if (ally.get() == Alliance.Blue) {
        return "B";
      }
    }
    else {
      return "";
    }
    return "";
  }

  private String getGameData(){
    String gameData = DriverStation.getGameSpecificMessage();
    gameData = gameData.toUpperCase();
    if(gameData.equals("L") || gameData.equals("R") || gameData.equals("C"))
    {
      return gameData;
    }
    return "";
  }


  private Intake createIntake() {
    Motor motor1 = new TalonFXMotor(29, "motor 1");
    Motor motor2 = new TalonFXMotor(41, "motor 2");
    return new Intake(motor1, motor2);
  }

  private Elevator createElevator() {
    //private final Elevator elevator = new Elevator(null, new SparkMaxBrushlessMotor(16, "left motor"), new SparkMaxBrushlessMotor(11, "right"));
    Motor leftMotor = new SparkMaxBrushlessMotor(16, "Elevator Left");
    Motor rightMotor = new SparkMaxBrushlessMotor(11, "Elevator Right");

    return new Elevator(leftMotor, rightMotor);
  }

  private Climber createClimb() {
    Motor leftMotor = new TalonFXMotor(58, "Left Climb Motor");
    Motor righMotor = new TalonFXMotor(3, "Right Climb Motor");

    return new Climber(leftMotor, righMotor);
  }

  private void mapControllers() {
    this.intake.setDefaultCommand(new IntakeCoral(intake, () -> manipulatorXbox.getLeftTriggerAxis() - manipulatorXbox.getRightTriggerAxis()));
    this.swivel.setDefaultCommand(new SwivelJoy(swivel, () -> manipulatorXbox.getLeftY()));
    this.elevator.setDefaultCommand(new RaiseElevator(elevator, () -> manipulatorXbox.getRightY()));
    
    this.climber.setDefaultCommand(new ClimbCage(climber, () -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis()));
    this.driverXbox.x().onTrue(new ClampCage(climber));
    this.driverXbox.b().onTrue(new UnclampCage(climber));
  }

  public void setMotorBrake(boolean brake)
  {
    driveBase.setMotorBrake(brake);
  }
}
