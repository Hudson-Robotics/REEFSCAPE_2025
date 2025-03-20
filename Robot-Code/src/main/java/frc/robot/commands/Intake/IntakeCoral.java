package frc.robot.commands.Intake;

import frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCoral extends Command {
    private final Intake intake;
    private final DoubleSupplier speed;

    public IntakeCoral(Intake intake, DoubleSupplier speed)
    {
        this.intake = intake;
        this.speed = speed;
        addRequirements(this.intake);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.setSpeed(speed.getAsDouble());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
