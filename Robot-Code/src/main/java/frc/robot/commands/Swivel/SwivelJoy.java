package frc.robot.commands.Swivel;

import frc.robot.subsystems.swivelL;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class SwivelJoy extends Command {
    private final swivelL swivel;
    private final DoubleSupplier speed;

    public SwivelJoy(swivelL intake, DoubleSupplier speed)
    {
        this.swivel = intake;
        this.speed = speed;
        addRequirements(this.swivel);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swivel.setSpeed(speed.getAsDouble());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swivel.setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
