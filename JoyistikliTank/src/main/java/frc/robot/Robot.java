package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Robot extends TimedRobot {
 
  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(1);
  private WPI_VictorSPX leftSlave = new WPI_VictorSPX(4);
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(2);

  private WPI_TalonSRX armMotor = new WPI_TalonSRX(5);
  private WPI_VictorSPX armSlave = new WPI_VictorSPX(6);

  private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(7);

  @SuppressWarnings("unused")
  private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid hatchIntake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); 

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  private Joystick driverJoystick = new Joystick(0);
  private Joystick operatorJoystick = new Joystick(1);

  private final double kDriveTick2Feet = 1.0 / 4096.0 * 6.0 * Math.PI / 12.0;
  
  private final double kArmTick2Deg = 360.0 / 512.0 * 26.0 / 42.0 * 18.0 / 60.0 * 18.0 / 84.0;

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm Encoder (deg)", armMotor.getSelectedSensorPosition() * kArmTick2Deg);
    SmartDashboard.putNumber("Left Drive Encoder (ft)", leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right Drive Encoder (ft)", rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);
  }

  @Override
  public void robotInit() {

    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    armMotor.setInverted(false);

    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(false);
    armMotor.setSensorPhase(true);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    armSlave.follow(armMotor);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    armSlave.setInverted(InvertType.FollowMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorPosition(0, 0, 10);

    int reverseLimitTicks = (int) (0.0 / kArmTick2Deg); // usually 0
    int forwardLimitTicks = (int) (175.0 / kArmTick2Deg);

    armMotor.configReverseSoftLimitThreshold(reverseLimitTicks, 10);
    armMotor.configForwardSoftLimitThreshold(forwardLimitTicks, 10);

    armMotor.configReverseSoftLimitEnable(true, 10);
    armMotor.configForwardSoftLimitEnable(true, 10);

    drive.setDeadband(0.05);
  }

  @Override
  public void autonomousInit() {
    enableMotors(true);
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorPosition(0, 0, 10);
  }

  @Override
  public void autonomousPeriodic() {
    double leftPosition = leftMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double rightPosition = rightMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) / 2.0;

    if (distance < 10.0) {
      drive.tankDrive(0.6, 0.6);
    } else {
      drive.tankDrive(0.0, 0.0);
    }
  }

  @Override
  public void teleopInit() {
    enableMotors(true);
  }

  @Override
  public void teleopPeriodic() {
    // driving
    double power = -driverJoystick.getRawAxis(1); 
    double turn = driverJoystick.getRawAxis(4);

    drive.arcadeDrive(power * 0.6, turn * 0.3);

    // arm control
    double armPower = -operatorJoystick.getRawAxis(1);
    if (Math.abs(armPower) < 0.05) {
      armPower = 0.0;
    }
    armPower *= 0.5;
    armMotor.set(ControlMode.PercentOutput, armPower);

    // roller control
    double rollerPower = 0.0;
    if (operatorJoystick.getRawButton(1)) {
      rollerPower = 1.0;
    } else if (operatorJoystick.getRawButton(2)) {
      rollerPower = -1.0;
    }
    rollerMotor.set(ControlMode.PercentOutput, rollerPower);

    // hatch intake
    if (operatorJoystick.getRawButton(3)) { 
      hatchIntake.set(Value.kReverse);
    } else {
      hatchIntake.set(Value.kForward);
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    enableMotors(false);
  }

  private void enableMotors(boolean on) {
    NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftSlave.setNeutralMode(mode);
    rightSlave.setNeutralMode(mode);
    armMotor.setNeutralMode(mode);
    armSlave.setNeutralMode(mode);
    rollerMotor.setNeutralMode(mode);
  }
}
