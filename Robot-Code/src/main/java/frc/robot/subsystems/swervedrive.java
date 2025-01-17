// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Interfaces.drivetrain;

public class swervedrive extends SubsystemBase  implements drivetrain {
    final SparkMax FrontleftCanSparksidways;
    final SparkMax FrontleftCanSparkforwordMax;
    final SparkMax frontrightCanSparkforwordMax;
  final SparkMax frontrightCanSparksidways;
  final SparkMax backrightCanSparkMaxBackwards;
  SparkMax BackrightCanSparkMaxBackwards;
    public swervedrive() {
      frontrightCanSparksidways = new SparkMax(5, MotorType.kBrushless);
      frontrightCanSparkforwordMax = new SparkMax(1, MotorType.kBrushless);
    //public swervedrive() {
      FrontleftCanSparksidways = new SparkMax(6, MotorType.kBrushless);
      FrontleftCanSparkforwordMax = new SparkMax(8, MotorType.kBrushless);
      backrightCanSparkMaxBackwards= new SparkMax (2, MotorType.kBrushless);
      //BackrightCanSparkMaxBackwards= new SparkMax (4, MotorType.kBrushless);
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
   SmartDashboard.putNumber("Frontright", .2); 
    frontrightCanSparksidways.set(.2);   
    frontrightCanSparkforwordMax.set(.2);
     SmartDashboard.putNumber("Frontleft", .2); 
   FrontleftCanSparksidways.set(.2);  
    FrontleftCanSparksidways.set(.2);
     SmartDashboard.putNumber("BackRight", .2); 
   BackrightCanSparkMaxBackwards.set(.2);  
    BackrightCanSparkMaxBackwards.set(.2);
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

