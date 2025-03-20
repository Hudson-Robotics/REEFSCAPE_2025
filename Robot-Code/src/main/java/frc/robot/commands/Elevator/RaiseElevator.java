package frc.robot.commands.Elevator;

import frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RaiseElevator extends Command {
    private final Elevator elevator;
    private final DoubleSupplier speed;

    public RaiseElevator(Elevator intake, DoubleSupplier speed)
    {
        this.elevator = intake;
        this.speed = speed;
        addRequirements(this.elevator);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.setSpeed(.2 * speed.getAsDouble());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
