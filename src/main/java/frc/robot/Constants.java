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

import com.revrobotics.spark.config.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
    public static final double kDirectionSlewRate = 1.2;
    public static final double kMagnitudeSlewRate = 1.8;
    public static final double kRotationalSlewRate = 2.0;
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
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
    public static final int kDrivingMotorPinionTeeth = 14;
    public static final boolean kTurningEncoderInverted = true;
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0;
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;
    public static final double kTurningEncoderPositionPIDMinInput = 0;
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;
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
    public static final int kDrivingMotorCurrentLimit = 50;
    public static final int kTurningMotorCurrentLimit = 20;
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
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
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
    public static final int ELEVATOR_MOTOR_ONE = 6;
    public static final int ELEVATOR_MOTOR_TWO = 11;
  }

  public static final class WristConstants {
    public static final int WRIST_MOTOR_ID = 41;
    public static final int MAX_WRIST_VOLTAGE = 12;
    public static final double WRIST_INTAKE_POSITION = 10.5;
    public static final double WRIST_SCORING_POSITION_L1 = 4;
    public static final double WRIST_SCORING_POSITION_L2 = 4.75;
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
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static String camera0Name = "limelight-bottom";
    public static String camera1Name = "limelight-top";
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;
    public static double linearStdDevBaseline = 0.02;
    public static double angularStdDevBaseline = 0.06;
    public static double[] cameraStdDevFactors = new double[] {1.0, 1.0};
    public static double linearStdDevMegatag2Factor = 0.5;
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
  }
}
