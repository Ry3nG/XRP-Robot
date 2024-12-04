package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class ArcadeDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;

  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_xaxisSpeedSupplier.get(), m_zaxisRotateSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
