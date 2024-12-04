package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LineFollowerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

public class ShowSpeed extends Command {
  private final Drivetrain m_drivetrain;

  double lastTimeStamp = 0;
  double leftEncoderDistance = 0;
  double rightEncoderDistance = 0;


  public ShowSpeed(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    lastTimeStamp = Timer.getFPGATimestamp();
    leftEncoderDistance = m_drivetrain.getLeftDistanceMilli();
    rightEncoderDistance = m_drivetrain.getRightDistanceMilli();
  }

  @Override
  public void execute() {
    double dt = Timer.getFPGATimestamp() - lastTimeStamp;
    lastTimeStamp = Timer.getFPGATimestamp();
    double dLeftEncoder = m_drivetrain.getLeftDistanceMilli() - leftEncoderDistance;
    double dRightEncoder = m_drivetrain.getRightDistanceMilli() - rightEncoderDistance;
    leftEncoderDistance = m_drivetrain.getLeftDistanceMilli();
    rightEncoderDistance = m_drivetrain.getRightDistanceMilli();

    SmartDashboard.putNumber("LeftVelocity", leftEncoderDistance/dt);
    SmartDashboard.putNumber("RightVelocity", rightEncoderDistance/dt);

    SmartDashboard.putNumber("LeftReflectance", m_drivetrain.getLeftReflectance());
    SmartDashboard.putNumber("RightReflectance", m_drivetrain.getRightReflectance());
  }
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
