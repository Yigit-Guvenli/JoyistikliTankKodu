package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {

    private final WPI_VictorSPX leftMotor = new WPI_VictorSPX(1);  
    private final WPI_VictorSPX rightMotor = new WPI_VictorSPX(2); 

    private final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

    public DriveSubsystem() {
        rightMotor.setInverted(true); // Sağ taraf ters çalışır
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
    }
}
