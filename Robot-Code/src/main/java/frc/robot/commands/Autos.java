// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

private final DriveSubsystem _drive;
private double startTime;
private final long tolerance = 12;

public AMoveEnd(DriveSubsystem drive) {
    _drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
}
// Called when the command is initially scheduled.
@Override
public void initialize(){
    startTime = Timer.getFPGATimestamp();
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute(){
    //_drive.drive(0.5, 0.0, 0.0, true);
}
// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    //_drive.setX();
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return (Timer.getFPGATimestamp()-startTime>=0.9);
}
