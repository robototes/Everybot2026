package frc.robot.subsystems.auto;

import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;

public class BLineTriggers {
  public static void registerTriggers(Subsystems s) {
    FollowPath.registerEventTrigger("Shoot", s.flywheels.runFlywheels());
    FollowPath.registerEventTrigger("Intake", s.intakeSubsystem.startIntake());
  }
}
