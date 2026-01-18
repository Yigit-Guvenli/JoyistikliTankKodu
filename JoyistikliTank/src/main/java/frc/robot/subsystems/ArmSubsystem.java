package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private  SparkMax armMotor = new SparkMax(3, null);
    private final Encoder encoder = new Encoder(1,2); 

    private PIDController pid = new PIDController(0.02, 0, 0); 

    private final double MIN_ANGLE = 0;
    private final double MAX_ANGLE = 180;

    public ArmSubsystem() {

        pid.setTolerance(1.0); 
    }

    public double getAngle() {
        return encoder.getDistance();
    }

    public void moveTo(double targetAngle) {

        if (targetAngle < MIN_ANGLE) targetAngle = MIN_ANGLE;
        if (targetAngle > MAX_ANGLE) targetAngle = MAX_ANGLE;

        double currentAngle = getAngle();
        double output = pid.calculate(currentAngle, targetAngle);  

        armMotor.set(output * -1);
    }

    public void stop() {
        armMotor.set(0);
    }
}