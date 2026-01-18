package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

public class TutCommand extends Command {

    private final ArmSubsystem arm;
    private final Joystick joystick;

    public TutCommand(ArmSubsystem arm, Joystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
    }

    public void execute() {

        if (joystick.getRawButton(4)) {
            arm.tut();
        }
        else if (joystick.getRawButton(5)) {
            arm.birak();
        }
        else {
            arm.tutmadur();
        }
    }
}
