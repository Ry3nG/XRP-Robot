package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.LineFollower;
import frc.robot.commands.ShowSpeed;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final Joystick m_controller = new Joystick(0);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());
  }

  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(1) * 0.8, () -> -m_controller.getRawAxis(0));
  }

  public Command getLineFollowerCommand() {
    return new LineFollower(m_drivetrain);
  }
}
