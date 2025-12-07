package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.TankDriveCommand;

public class RobotContainer {

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final Joystick joystick = new Joystick(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        driveSubsystem.setDefaultCommand(
            new TankDriveCommand(
                driveSubsystem,
                () -> -joystick.getRawAxis(1),  // Sol
                () -> -joystick.getRawAxis(5)   // Sağ
            )
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
