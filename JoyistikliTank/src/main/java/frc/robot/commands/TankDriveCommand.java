package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class TankDriveCommand extends Command {

    private final DriveSubsystem drive;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier rightInput;

    public TankDriveCommand(
        DriveSubsystem subsystem,
        DoubleSupplier leftInput,
        DoubleSupplier rightInput
    ) {
        this.drive = subsystem;
        this.leftInput = leftInput;
        this.rightInput = rightInput;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.tankDrive(leftInput.getAsDouble(), rightInput.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        drive.tankDrive(0, 0);
    }
}
