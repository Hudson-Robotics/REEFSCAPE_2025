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

//package frc.robot;
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.revrobotics.spark.config.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

      public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  
   public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 1;

    public static final int kIntakeCanId = 10;

    public static final int kGyroCanId = 9;

    public static final boolean kGyroReversed = false;
  }

 public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kOperatorControllerPort = 1;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class SparkMaxIDs {
    public static final int FRONT_LEFT_DRIVE = 13;
    public static final int FRONT_LEFT_STEER = 1;

    public static final int FRONT_RIGHT_DRIVE = 31;
    public static final int FRONT_RIGHT_STEER = 8;

    public static final int BACK_LEFT_DRIVE = 9;
    public static final int BACK_LEFT_STEER = 5;

    public static final int BACK_RIGHT_DRIVE = 3;
    public static final int BACK_RIGHT_STEER = 14;

    //public static final int UP_MOTOR_CLIMB_ONE = 9;
    //public static final int UP_MOTOR_CLIMB_TWO = 10;

    //public static final int INTAKE_MOTOR_ONE = 11;
    //public static final int INTAKE_MOTOR_TWO = 12;

    public static final int ELEVATOR_MOTOR_ONE = 6;
    public static final int ELEVATOR_MOTOR_TWO = 11;
    
    //public static final int SWIVEL_MOTOR_UP = 15;
}

public static final class WristConstants {
    public static final int WRIST_MOTOR_ID = 41;
    public static final int MAX_WRIST_VOLTAGE = 12;

    public static final double WRIST_INTAKE_POSITION = 10.5;
    public static final double WRIST_SCORING_POSITION_L1 = 4;
    public static final double WRIST_SCORING_POSITION_L2 = 4.75; // L2 and L3 should be same
    public static final double WRIST_SCORING_POSITION_L4 = 5.75;
    public static final double WRIST_PERPENDICULAR_POSITION = 7.5;
    public static final double WRIST_SCORING_POSITION_NET = 11;

    public static double kP = .65;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 0.11237;
    public static double kV = 0.56387;
    public static double kA = 0.041488;
    public static double kG = 0.76416;
  }

  public final class VisionConstants {
    public static final double alignSpeed = .4;
    public static final double alignRange = 3;
    public static final double closeAlignSpeed = .25;
    public static final double closeAlignRange = 1;

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-bottom";
    public static String camera1Name = "limelight-top";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

}


public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = 
      new CANBus (TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
        Math.max(
          Math.hybot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          Math.hybot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)
        ),
      Math.max(
        Math.hybot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        Math.hybot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
      ));

  // PathPlanner configure constants
  private static final double ROBOT_MASS_KG = 48.0; 
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
            TunerConstants.FrontLeft.WheelRadius,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            WHEEL_COF,
            DCMotor.getNEO(kDriverControllerPort)(1),
                .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
            TunerConstants.FrontLeft.SlipCurrent, 1),
            getModuleTranslations());
          
  static final Lock OdometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLOgged();
  public final Module[] modules = new Module[4];
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Gyro Disconnected, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
     this.gyroIO = gyroIO;
      modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft, PP_CONFIG);
      modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight, PP_CONFIG);
      modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft, PP_CONFIG);
      modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight, PP_CONFIG);

      // Usage reporting for swerve template
      HAL.report(tResourceType.KresourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

      //start odometry thread
      PhoenixOdometryThread.getInstance().start();

      // Sweve Visualization for dashboard
      SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Swerve Drive");

            builder.addDoubleProperty("Front Left Drive Motor", modules[0].getDriveMotor()::getVelocity, null);
            builder.addDoubleProperty("Front Left Steer Motor", modules[0].getSteerMotor()::getVelocity, null);
            builder.addDoubleProperty("Front Right Drive Motor", modules[1].getDriveMotor()::getVelocity, null);
            builder.addDoubleProperty("Front Right Steer Motor", modules[1].getSteerMotor()::getVelocity, null);
            builder.addDoubleProperty("Back Left Drive Motor", modules[2].getDriveMotor()::getVelocity, null);
            builder.addDoubleProperty("Back Left Steer Motor", modules[2].getSteerMotor()::getVelocity, null);
            builder.addDoubleProperty("Back Right Drive Motor", modules[3].getDriveMotor()::getVelocity, null);
            builder.addDoubleProperty("Back Right Steer Motor", modules[3].getSteerMotor()::getVelocity, null);

            builder.addDoubleProperty("Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
            builder.addDoubleProperty(
              "Front Left Velocity", () -> -modules[0].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
            builder.addDoubleProperty(
              "Front Right Velocity", () -> -modules[1].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
            builder.addDoubleProperty(
              "Back Left Velocity", () -> -modules[2].getVelocityMetersPerSec(), null);
            builder.addDoubleProperty("Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
            builder.addDoubleProperty(
              "Back Right Velocity", () -> -modules[3].getVelocityMetersPerSec(), null);
              builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
          }
        }
      );
  
  AutoBuilder.configure(
    this::getPose,
    this::setPose,
    this::getChassisSpeeds,
    this::runVelocity,
    new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
    PP_CONFIG,
    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
    this);
  Pathfinding.setPathfinder(new LocalADStarAK());
  PathPlannerLogging.setLogActivePathCallback(
    (activePath) -> {
      Logger.recordOutput(
        "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    });
  PathPlannerLogging.setLogTargetPoseCallback(
    (targetPose) -> {
      Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    });

    // Configure Sysid
    sysId =
        new SysIdRoutine(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(volts)), null, this));
   );

   @Override
   public void periodic() {
     odometryLock.lock(); // Prevents odometry updates while reading data
     gyroIO.updateInputs(gyroInputs);
     Logger.processInputs("Drive/Gyro", gyroInputs);
     for (var module : modules) {
       module.periodic();
     }
     odometryLock.unlock();
 
     // Stop moving when disabled
     if (DriverStation.isDisabled()) {
       for (var module : modules) {
         module.stop();
       }
     }
 
     // Log empty setpoint states when disabled
     if (DriverStation.isDisabled()) {
       Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
       Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
     }
 
     // Update odometry
     double[] sampleTimestamps =
         modules[0].getOdometryTimestamps(); // All signals are sampled together
     int sampleCount = sampleTimestamps.length;
     for (int i = 0; i < sampleCount; i++) {
       // Read wheel positions and deltas from each module
       SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
       SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
       for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
         modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
         moduleDeltas[moduleIndex] =
             new SwerveModulePosition(
                 modulePositions[moduleIndex].distanceMeters
                     - lastModulePositions[moduleIndex].distanceMeters,
                 modulePositions[moduleIndex].angle);
         lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
       }
 
       // Update gyro angle
       if (gyroInputs.connected) {
         // Use the real gyro angle
         rawGyroRotation = gyroInputs.odometryYawPositions[i];
       } else {
         // Use the angle delta from the kinematics and module deltas
         Twist2d twist = kinematics.toTwist2d(moduleDeltas);
         rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
       }
 
       // Apply update
       poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
     }
 
     // Update gyro alert
     gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
   }
 
   /**
    * Runs the drive at the desired velocity.
    *
    * @param speeds Speeds in meters/sec
    */
   public void runVelocity(ChassisSpeeds speeds) {
     // Calculate module setpoints
     ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
     SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
     SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);
 
     // Log unoptimized setpoints and setpoint speeds
     Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
     Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
 
     // Send setpoints to modules
     for (int i = 0; i < 4; i++) {
       modules[i].runSetpoint(setpointStates[i]);
     }
 
     // Log optimized setpoints (runSetpoint mutates each state)
     Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
   }
 
   /** Runs the drive in a straight line with the specified drive output. */
   public void runCharacterization(double output) {
     for (int i = 0; i < 4; i++) {
       modules[i].runCharacterization(output);
     }
   }
 
   /** Stops the drive. */
   public void stop() {
     runVelocity(new ChassisSpeeds());
   }
 
   /**
    * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
    * return to their normal orientations the next time a nonzero velocity is requested.
    */
   public void stopWithX() {
     Rotation2d[] headings = new Rotation2d[4];
     for (int i = 0; i < 4; i++) {
       headings[i] = getModuleTranslations()[i].getAngle();
     }
     kinematics.resetHeadings(headings);
     stop();
   }
 
   /** Returns a command to run a quasistatic test in the specified direction. */
   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
     return run(() -> runCharacterization(0.0))
         .withTimeout(1.0)
         .andThen(sysId.quasistatic(direction));
   }
 
   /** Returns a command to run a dynamic test in the specified direction. */
   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
     return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
   }
 
   /** Returns the module states (turn angles and drive velocities) for all of the modules. */
   @AutoLogOutput(key = "SwerveStates/Measured")
   private SwerveModuleState[] getModuleStates() {
     SwerveModuleState[] states = new SwerveModuleState[4];
     for (int i = 0; i < 4; i++) {
       states[i] = modules[i].getState();
     }
     return states;
   }
 
   /** Returns the module positions (turn angles and drive positions) for all of the modules. */
   private SwerveModulePosition[] getModulePositions() {
     SwerveModulePosition[] states = new SwerveModulePosition[4];
     for (int i = 0; i < 4; i++) {
       states[i] = modules[i].getPosition();
     }
     return states;
   }
 
   /** Returns the measured chassis speeds of the robot. */
   @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
   private ChassisSpeeds getChassisSpeeds() {
     return kinematics.toChassisSpeeds(getModuleStates());
   }
 
   /** Returns the position of each module in radians. */
   public double[] getWheelRadiusCharacterizationPositions() {
     double[] values = new double[4];
     for (int i = 0; i < 4; i++) {
       values[i] = modules[i].getWheelRadiusCharacterizationPosition();
     }
     return values;
   }
 
   /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
   public double getFFCharacterizationVelocity() {
     double output = 0.0;
     for (int i = 0; i < 4; i++) {
       output += modules[i].getFFCharacterizationVelocity() / 4.0;
     }
     return output;
   }
 
   /** Returns the current odometry pose. */
   @AutoLogOutput(key = "Odometry/Robot")
   public Pose2d getPose() {
     return poseEstimator.getEstimatedPosition();
   }
 
   /** Returns the current odometry rotation. */
   public Rotation2d getRotation() {
     return getPose().getRotation();
   }
 
   /** Resets the current odometry pose. */
   public void setPose(Pose2d pose) {
     poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
   }
 
   /** Adds a new timestamped vision measurement. */
   public void addVisionMeasurement(
       Pose2d visionRobotPoseMeters,
       double timestampSeconds,
       Matrix<N3, N1> visionMeasurementStdDevs) {
     poseEstimator.addVisionMeasurement(
         visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
   }
 
   /** Returns the maximum linear speed in meters per sec. */
   public double getMaxLinearSpeedMetersPerSec() {
     return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
   }
 
   /** Returns the maximum angular speed in radians per sec. */
   public double getMaxAngularSpeedRadPerSec() {
     return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
   }
 
   /** Returns an array of module translations. */
   public static Translation2d[] getModuleTranslations() {
     return new Translation2d[] {
       new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
       new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
       new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
       new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
     };
   }
 }
 
      sysId = new SysIdRoutine(this);
      poseEstimator.setModulePositions(getModulePositions());
      poseEstimator.setGyroAngle(gyroIO.getAngle());
      poseEstimator.setGyroRate(gyroIO.getRate());
      poseEstimator.setModuleVelocities(getModuleVelocities());
      poseEstimator.setModulePositions(getModulePositions());
      poseEstimator.setGyroAngle(gyroIO.getAngle());
      poseEstimator.setGyroRate(gyroIO.getRate());
      poseEstimator.setModuleVelocities(getModuleVelocities());
      poseEstimator.setModulePositions(getModulePositions());
      poseEstimator.setGyroAngle(gyroIO.getAngle());
      poseEstimator.setGyroRate(gyroIO.getRate());
      poseEstimator.setModuleVelocities(getModuleVelocities());
      poseEstimator.setModulePositions(getModulePositions());
      poseEstimator.setGyroAngle(gyroIO.getAngle());
      poseEstimator.setGyroRate(gyroIO.getRate());
      poseEstimator.setModuleVelocities(getModuleVelocities());
      poseEstimator.setModulePositions(getModulePositions());
      poseEstimator.setGyroAngle(gyroIO.getAngle());
      poseEstimator.setGyroRate(gyroIO.getRate());
      poseEstimator.setModuleVelocities(getModuleVelocities());
      poseEstimator.setModulePositions(getModulePositions());
      poseEstimator.setGyroAngle(gyroIO.getAngle());
      poseEstimator.setGyroRate(gyroIO.getRate());
      poseEstimator.setModuleVelocities(getModuleVelocities());
      poseEstimator.setModulePositions(getModulePositions());
      poseEstimator.setGyroAngle(gyroIO.getAngle());
      poseEstimator.setGyroRate(gyroIO.getRate());
      poseEstimator.setModuleVelocities(getModuleVelocities());
      poseEstimator.setModulePositions(getModulePositions());
      poseEstimator.setGyroAngle(gyroIO.getAngle());
      poseEstimator.set
    }
  )
  private final SwerveDrive drivetrain = new SwerveDrive();
  private final GyroIO gyro = new GyroIO();
  private final ModuleIO moduleIO = new ModuleIO();
  private final ModuleIOSim moduleIOSim = new ModuleIOSim();
  private final SparkMaxBrushlessEncoderMotor frontLeftDrive = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_LEFT_DRIVE, "FrontLeftDrive", false, 1);
  private final SparkMaxBrushlessEncoderMotor frontLeftSteer = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_LEFT_STEER, "FrontLeftSteer", true, 360);
  private final SparkMaxBrushlessEncoderMotor frontRightDrive = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_RIGHT_DRIVE, "frontRightDrive", false, 1);
  private final SparkMaxBrushlessEncoderMotor frontRightSteer = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_RIGHT_STEER, "frontRightSteer", true, 360);
  private final SparkMaxBrushlessEncoderMotor backLeftDrive = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.BACK_LEFT_DRIVE, "backLeftDrive", false, 1);
  private final SparkMaxBrushlessEncoderMotor backLeftSteer = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.BACK_LEFT_STEER, "backLeftSteer", true, 360);
  private final SparkMaxBrushlessEncoderMotor backRightDrive = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_LEFT_DRIVE, "backRightDrive", false, 1);
  private final SparkMaxBrushlessEncoderMotor backRightSteer = new SparkMaxBrushlessEncoderMotor(SparkMaxIDs.FRONT_LEFT_STEER, "backRightSteer", true, 360);
  private final SwerveModule frontLeft = new SwerveModule(frontLeftDrive, frontLeftSteer);
  private final SwerveModule frontRight = new SwerveModule(frontRightDrive, frontRightSteer);
  private final SwerveModule backLeft = new SwerveModule(backLeftDrive, backLeftSteer);
  private final SwerveModule backRight = new SwerveModule(backRightDrive, backRightSteer);
  private final SwerveDrive drivetrain = new SwerveDrive(frontLeft, frontRight, backLeft, backRight);
  private final SwerveDrive drivetrain = new SwerveDrive();
  private final SwerveDrive drivetrain = new SwerveDrive();

