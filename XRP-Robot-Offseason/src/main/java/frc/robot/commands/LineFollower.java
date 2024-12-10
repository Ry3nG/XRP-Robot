package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LineFollowerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LineFollower extends Command {
  private final Drivetrain m_drivetrain;
  public LineFollower(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if ((m_drivetrain.getLeftReflectance() <= LineFollowerConstants.kDarkness)
      && (m_drivetrain.getRightReflectance() <= LineFollowerConstants.kDarkness)) {
      m_drivetrain.arcadeDriveSpeed(0.425, 0.025);
    } else if ((m_drivetrain.getLeftReflectance() >= LineFollowerConstants.kDarkness)
      && (m_drivetrain.getRightReflectance() <= LineFollowerConstants.kDarkness)) {
      m_drivetrain.arcadeDriveSpeed(3.5, 1.5);
    } else if ((m_drivetrain.getLeftReflectance() <= LineFollowerConstants.kDarkness)
      && (m_drivetrain.getRightReflectance() >= LineFollowerConstants.kDarkness)) {
      m_drivetrain.arcadeDriveSpeed(4.5, 2.5);
    } else if ((m_drivetrain.getLeftReflectance() >= LineFollowerConstants.kDarkness)
      && (m_drivetrain.getRightReflectance() >= LineFollowerConstants.kDarkness)) {
      m_drivetrain.arcadeDriveSpeed(0, 0);
    }
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
