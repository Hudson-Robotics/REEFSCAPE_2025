package frc.robot.commands.Climb;

import frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ClampCage extends Command {
    private final Climber climb;

    public ClampCage(Climber climb)
    {
        this.climb = climb;
        addRequirements(this.climb);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climb.clamp();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //climb.climb(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
