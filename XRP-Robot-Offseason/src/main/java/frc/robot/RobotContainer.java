package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDriveSpeed;
import frc.robot.commands.LineFollower;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final Joystick m_controller = new Joystick(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
   // m_drivetrain.setDefaultCommand(new ShowSpeed(m_drivetrain));
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());
  }

  public Command getArcadeDriveCommand() {
    return new ArcadeDriveSpeed(
        m_drivetrain, () -> -m_controller.getRawAxis(OperatorConstants.kXaxisNumber), () -> -m_controller.getRawAxis(OperatorConstants.kZaxisNumber));
  }

  public Command getLineFollowerCommand() {
    return new LineFollower(m_drivetrain);
  }
}
