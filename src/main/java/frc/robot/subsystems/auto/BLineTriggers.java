package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;

public class BLineTriggers {
  public static void registerTriggers(Subsystems s) {
    FollowPath.registerEventTrigger("Shoot",AutoCommands.launch(s));
    FollowPath.registerEventTrigger("Intake", AutoCommands.intake((s)));
     FollowPath.registerEventTrigger("Wait", Commands.waitSeconds(5));
  }
}
