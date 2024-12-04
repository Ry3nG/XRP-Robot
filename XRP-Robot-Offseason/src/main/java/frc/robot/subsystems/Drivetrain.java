package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.xrp.XRPReflectanceSensor;

public class Drivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  private final XRPGyro m_gyro = new XRPGyro();

  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private final XRPReflectanceSensor m_reflectance = new XRPReflectanceSensor();

  public Drivetrain() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    m_rightMotor.setInverted(true);
    m_leftMotor.setInverted(false);

    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }

  public void setLeftMotorSpeed(double speed) {
    m_leftMotor.set(speed);
  }

  public void setRightMotorSpeed(double speed) {
    m_rightMotor.set(speed);
  }


  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  public double getAccelX() {
    return m_accelerometer.getX();
  }

  public double getAccelY() {
    return m_accelerometer.getY();
  }

  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public double getLeftReflectance() {
    return m_reflectance.getLeftReflectanceValue();
  }

  public double getRightReflectance() {
    return m_reflectance.getRightReflectanceValue();
  }

  @Override
  public void periodic() {}
}
