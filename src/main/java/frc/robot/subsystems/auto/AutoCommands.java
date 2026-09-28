package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;

public class AutoCommands {
  public static Command launch(Subsystems s, Double timeout) {
    return Commands.runOnce(() -> s.flywheels.runFlywheels().withTimeout(timeout));
  }

  public static Command launch(Subsystems s) {
    return Commands.runOnce(() -> s.flywheels.runFlywheels());
  }

  public static Command intake(Subsystems s) {
    return Commands.runOnce(() -> s.intakeSubsystem.startIntake());
  }
}
