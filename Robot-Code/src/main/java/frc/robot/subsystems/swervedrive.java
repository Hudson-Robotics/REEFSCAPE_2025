// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.drivetrain;

public class swervedrive extends SubsystemBase  implements drivetrain {
  //xbox
  private XboxController controller; 
   //sparkmax
    final SparkMax frontrightCanSparkforwordMax1;
 final SparkMax frontrightCanSparksidways5;
  final SparkMax FrontleftCanSparksidways6;
   final SparkMax FrontleftCanSparkforwordMax8;
   SparkMax backrightCanSparkMaxBackwards4;
   final SparkMax BackrightCanSparkMaxBackwards2;
   final SparkMax BackleftCanSparkmaxwards7;
   final SparkMax BackleftCanSparkmaxwards3;
       public swervedrive() { 
         controller = new XboxController(1);
   
         frontrightCanSparksidways5 = new SparkMax(5, MotorType.kBrushless);
         frontrightCanSparkforwordMax1 = new SparkMax(1, MotorType.kBrushless);
   
       FrontleftCanSparksidways6 = new SparkMax(6, MotorType.kBrushless);
       FrontleftCanSparkforwordMax8 = new SparkMax(8, MotorType.kBrushless);
       backrightCanSparkMaxBackwards4= new SparkMax (4, MotorType.kBrushless);
       BackrightCanSparkMaxBackwards2= new SparkMax (4, MotorType.kBrushless);
       BackleftCanSparkmaxwards7= new SparkMax (7, MotorType.kBrushless);
       BackleftCanSparkmaxwards3= new SparkMax (3, MotorType.kBrushless);
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
      // Get the state of the A button (boolean)
      boolean aButtonPressed = controller.getAButton();
      SmartDashboard.putBoolean("A Button Pressed", aButtonPressed);
      double leftY = controller.getLeftY();
      SmartDashboard.putNumber("Left Joystick Y", leftY);
      double rightX = controller.getRightX();
      SmartDashboard.putNumber("Right Joystick X", rightX);
      
      double driveMotorSpeed = leftY;
      frontrightCanSparkforwordMax1.set(driveMotorSpeed);
      SmartDashboard.putNumber("Drive Motor Speed", driveMotorSpeed);
      double steerMotorSpeed = rightX;
      frontrightCanSparksidways5.set(steerMotorSpeed);
      SmartDashboard.putNumber("Steer Motor Speed", steerMotorSpeed);
       backrightCanSparkMaxBackwards4= new SparkMax (4, MotorType.kBrushless);
  
    BackrightCanSparkMaxBackwards2.set(steerMotorSpeed);
   SmartDashboard.putNumber("Steer Motor Speed", steerMotorSpeed);

   FrontleftCanSparkforwordMax8.set(steerMotorSpeed);
   SmartDashboard.putNumber("Steer Motor Speed", steerMotorSpeed);
 FrontleftCanSparkforwordMax8.set(steerMotorSpeed);
   SmartDashboard.putNumber("Steer Motor Speed", steerMotorSpeed);
   FrontleftCanSparksidways6.set(steerMotorSpeed);
   SmartDashboard.putNumber("Steer Motor Speed", steerMotorSpeed);
   //speed for sparkmaxs:)
    SmartDashboard.putNumber("Frontright", .05); 
    frontrightCanSparksidways5.set(.5);   
    frontrightCanSparkforwordMax1.set(.05);
  SmartDashboard.putNumber("Frontleft", .5); 
  FrontleftCanSparksidways6.set(.05);  
  FrontleftCanSparkforwordMax8.set(.2);
  SmartDashboard.putNumber("BackRight", .5); 
  BackrightCanSparkMaxBackwards2.set(.2);  
  backrightCanSparkMaxBackwards4.set(.05);
  SmartDashboard.putNumber("backleft", .5);
   BackleftCanSparkmaxwards3.set(.2);
  BackleftCanSparkmaxwards7.set(.05);
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


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.drivetrain;

import frc.robot.HelperClasses.NeoSwerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

public class swervedrive extends SubsystemBase  implements drivetrain {
  // xbox
  private XboxController controller; 
  
  // sparkmax
  // final SparkMax frontrightCanSparkforwordMax1;
  // final SparkMax frontrightCanSparksidways5;
  // final SparkMax FrontleftCanSparksidways6;
  // final SparkMax FrontleftCanSparkforwordMax8;
  // final SparkMax backrightCanSparkMaxBackwards4;
  // final SparkMax BackrightCanSparkMaxBackwards2;
  // final SparkMax BackleftCanSparkmaxwards7;
  // final SparkMax BackleftCanSparkmaxwards3;

  // Swerve Modules
  final NeoSwerve frontLeftSwerveModule;
  final NeoSwerve frontRightSwerveModule;
  final NeoSwerve backLeftSwerveModule;
  final NeoSwerve backRightSwerveModule;

  // Nav X
  final AHRS navX;

  // DriveTrain Kinematics
  final SwerveDriveKinematics DriveTrainKinematics; // this will be cleaned

  // Size
  final double LENGTH_IN_METERS = .381;

  public swervedrive() { 
    controller = new XboxController(1);

    // frontrightCanSparksidways5 = new SparkMax(5, MotorType.kBrushless); // GOOD JOB on naming these, it helped
    // frontrightCanSparkforwordMax1 = new SparkMax(1, MotorType.kBrushless);
    frontRightSwerveModule = new NeoSwerve(5,1,"frontRightSwerveDrive");

    // FrontleftCanSparksidways6 = new SparkMax(6, MotorType.kBrushless);
    // FrontleftCanSparkforwordMax8 = new SparkMax(8, MotorType.kBrushless);
    frontLeftSwerveModule = new NeoSwerve(6,8,"frontRightSwerveDrive");

    // backrightCanSparkMaxBackwards4= new SparkMax (4, MotorType.kBrushless); // SMH Names were not descriptive
    // BackrightCanSparkMaxBackwards2= new SparkMax (4, MotorType.kBrushless);
    backRightSwerveModule = new NeoSwerve(4,2,"frontRightSwerveDrive");

    // BackleftCanSparkmaxwards7= new SparkMax (7, MotorType.kBrushless);
    // BackleftCanSparkmaxwards3= new SparkMax (3, MotorType.kBrushless);
    backLeftSwerveModule = new NeoSwerve(7,3,"frontRightSwerveDrive");

    navX = new AHRS(NavXComType.kMXP_SPI);

    Translation2d frontLeftLocation = new Translation2d(-LENGTH_IN_METERS, LENGTH_IN_METERS);
    Translation2d frontRightLocation = new Translation2d(LENGTH_IN_METERS, LENGTH_IN_METERS);
    Translation2d backLeftLocation = new Translation2d(-LENGTH_IN_METERS, -LENGTH_IN_METERS);
    Translation2d backRightLocation = new Translation2d(LENGTH_IN_METERS, -LENGTH_IN_METERS);

    DriveTrainKinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
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
    // Get the state of the A button (boolean)
    boolean aButtonPressed = controller.getAButton();
    SmartDashboard.putBoolean("A Button Pressed", aButtonPressed);

    double leftX = controller.getLeftX();
    SmartDashboard.putNumber("Left Joystick X", leftX);

    double leftY = controller.getLeftY();
    SmartDashboard.putNumber("Left Joystick Y", leftY);

    double rightX = controller.getRightX();
    SmartDashboard.putNumber("Right Joystick X", rightX);
    
    var swerveModuleStates = this.DriveTrainKinematics.toSwerveModuleStates(new ChassisSpeeds(leftX, leftY, rightX)); // creating a new here is bad cuz it is taking up space every periodic call

    this.frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    this.frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
    this.backLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
    this.backRightSwerveModule.setDesiredState(swerveModuleStates[3]);

    // double driveMotorSpeed = leftY;
    // frontrightCanSparkforwordMax1.set(driveMotorSpeed);
    // SmartDashboard.putNumber("Drive Motor Speed", driveMotorSpeed);
    // double steerMotorSpeed = rightX;
    // frontrightCanSparksidways5.set(steerMotorSpeed);
    // SmartDashboard.putNumber("Steer Motor Speed", steerMotorSpeed);
    // // backrightCanSparkMaxBackwards4= new SparkMax (4, MotorType.kBrushless);

    // BackrightCanSparkMaxBackwards2.set(steerMotorSpeed);
    // SmartDashboard.putNumber("Steer Motor Speed", steerMotorSpeed);

    // FrontleftCanSparkforwordMax8.set(steerMotorSpeed);
    // SmartDashboard.putNumber("Steer Motor Speed", steerMotorSpeed);
    // FrontleftCanSparkforwordMax8.set(steerMotorSpeed);
    // SmartDashboard.putNumber("Steer Motor Speed", steerMotorSpeed);
    // FrontleftCanSparksidways6.set(steerMotorSpeed);
    // SmartDashboard.putNumber("Steer Motor Speed", steerMotorSpeed);
    // //speed for sparkmaxs:)
    // SmartDashboard.putNumber("Frontright", .05); 
    // frontrightCanSparksidways5.set(.5);   
    // frontrightCanSparkforwordMax1.set(.05);
    // SmartDashboard.putNumber("Frontleft", .5); 
    // FrontleftCanSparksidways6.set(.05);  
    // FrontleftCanSparkforwordMax8.set(.2);
    // SmartDashboard.putNumber("BackRight", .5); 
    // BackrightCanSparkMaxBackwards2.set(.2);  
    // backrightCanSparkMaxBackwards4.set(.05);
    // SmartDashboard.putNumber("backleft", .5);
    // BackleftCanSparkmaxwards3.set(.2);
    // BackleftCanSparkmaxwards7.set(.05);
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

