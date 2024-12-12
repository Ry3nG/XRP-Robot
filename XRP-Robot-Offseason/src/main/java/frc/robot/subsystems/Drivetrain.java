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

    m_leftEncoder.setDistancePerPulse(0.5);
    m_rightEncoder.setDistancePerPulse(0.5);
    resetEncoders();
  }

  public void setLeftMotorVoltage(double voltage) {
    m_leftMotor.set(voltage);
  }

  public void setRightMotorVoltage(double voltage) {
    m_rightMotor.set(voltage);
  }

  public double calculateXaxisVoltage(double xaxisSpeed, double zaxisRotate) {
    return m_leftController.calculate(m_leftEncoder.getRate(), ((xaxisSpeed - zaxisRotate)*300));
  }

  public double calculateZaxisVoltage(double xaxisSpeed, double zaxisRotate) {
    return m_rightController.calculate(m_rightEncoder.getRate(), ((xaxisSpeed + zaxisRotate)*400));
  }

  public void arcadeDriveSpeed(double xaxisSpeed, double zaxisRotate) {
/*    if ((xaxisSpeed<0.05) || (zaxisRotate<0.05)) {
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    } else { */
    m_leftMotor.set(calculateXaxisVoltage(xaxisSpeed, zaxisRotate)*0.2);
    
    m_rightMotor.set(calculateZaxisVoltage(xaxisSpeed, zaxisRotate)*0.2);
    
  }
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
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
