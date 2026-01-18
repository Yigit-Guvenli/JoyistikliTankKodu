package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {

    private final ArmSubsystem arm;
    private final Joystick joystick;

    public ArmCommand(ArmSubsystem arm, Joystick joystick) {
        this.arm = arm;
        this.joystick = joystick;
    }

    public void execute() {

        if (joystick.getRawButton(1)) {
            arm.moveTo(45);
        }
        else if (joystick.getRawButton(2)) {
            arm.moveTo(90);
        }
        else if (joystick.getRawButton(3)) {
            arm.moveTo(180);
        }
        else {
            arm.stop();
        }
    }


}