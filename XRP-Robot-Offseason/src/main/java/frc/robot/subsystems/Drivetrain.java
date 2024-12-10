package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj.xrp.XRPReflectanceSensor;
import edu.wpi.first.math.controller.PIDController;

public class Drivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterMilli = 60; // 60 mm

  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  private final XRPReflectanceSensor m_reflectance = new XRPReflectanceSensor();
  
  private final PIDController m_leftController = new PIDController(DrivetrainConstants.kleftP, DrivetrainConstants.kleftI, DrivetrainConstants.kleftD);
  private final PIDController m_rightController = new PIDController(DrivetrainConstants.krightP, DrivetrainConstants.krightI, DrivetrainConstants.krightD);

  public Drivetrain() {
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    m_rightMotor.setInverted(true);
    m_leftMotor.setInverted(false);

    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMilli) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMilli) / kCountsPerRevolution);
    resetEncoders();
  }

  public void setLeftMotorVoltage(double voltage) {
    m_leftMotor.set(voltage);
  }

  public void setRightMotorVoltage(double voltage) {
    m_rightMotor.set(voltage);
  }

  public double calculateLeftMotorVoltage(double leftMotorSpeed) {
    return m_leftController.calculate(m_leftEncoder.getRate(), leftMotorSpeed);
  }

  public double calculateRightMotorVoltage(double rightMotorSpeed) {
    return m_rightController.calculate(m_rightEncoder.getRate(), rightMotorSpeed);
  }

  public double calculateXaxisVoltage(double leftMotorSpeed, double rightMotorSpeed) {
    return (calculateLeftMotorVoltage(leftMotorSpeed)+calculateRightMotorVoltage(rightMotorSpeed))/2;
  }

  public double calculateZaxisVoltage(double leftMotorSpeed, double rightMotorSpeed) {
    return (calculateLeftMotorVoltage(leftMotorSpeed)-calculateRightMotorVoltage(rightMotorSpeed))/2;
  }

  public void arcadeDriveSpeed(double leftMotorSpeed, double rightMotorSpeed) {
    m_diffDrive.arcadeDrive(calculateXaxisVoltage(leftMotorSpeed, rightMotorSpeed), calculateXaxisVoltage(leftMotorSpeed, rightMotorSpeed));
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftEncoderRate() {
    return m_leftEncoder.getRate();
  }

  public double getRightEncoderRate() {
    return m_rightEncoder.getRate();
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
