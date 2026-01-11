package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private VictorSPX armMotor = new VictorSPX(3);
    private CANCoder armEncoder = new CANCoder(1); 

    private PIDController pid = new PIDController(0.02, 0, 0); 

    private final double MIN_ANGLE = 0;
    private final double MAX_ANGLE = 180;

    public ArmSubsystem() {
        armMotor.setInverted(false);

        pid.setTolerance(1.0); 
    }

    public double getAngle() {
        return armEncoder.getAbsolutePosition();
    }

    public void moveTo(double targetAngle) {

        if (targetAngle < MIN_ANGLE) targetAngle = MIN_ANGLE;
        if (targetAngle > MAX_ANGLE) targetAngle = MAX_ANGLE;

        double currentAngle = getAngle();
        double output = pid.calculate(currentAngle, targetAngle);  

        armMotor.set(ControlMode.PercentOutput, output);
    }

    public void stop() {
        armMotor.set(ControlMode.PercentOutput, 0);
    }
}