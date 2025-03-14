// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static Command exampleAuto(ExampleSubsystem subsystem) {
        return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    }

    public static Command moveEndAuto(DriveSubsystem drive) {
        return new MoveEndCommand(drive);
    }

    private Autos() {
        // Prevent instantiation
    }

    public static class MoveEndCommand extends CommandBase {

        private final DriveSubsystem drive;
        private double startTime;

        public MoveEndCommand(DriveSubsystem drive) {
            this.drive = drive;
            // Use addRequirements() here to declare subsystem dependencies.
            addRequirements(drive);
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            startTime = Timer.getFPGATimestamp();
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            drive.drive(0.5, 0.0, 0.0, true);
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            drive.setX();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return (Timer.getFPGATimestamp() - startTime >= 0.9);
        }
    }
}
