package frc.robot.subsystems.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;

public class BLineTriggers {
  public static void registerTriggers(Subsystems s) {
    FollowPath.registerEventTrigger("Shoot",AutoCommands.launch(s).withName("SHOOTING"));
    FollowPath.registerEventTrigger("Intake", AutoCommands.intake((s)).withName("INTAKING"));
     FollowPath.registerEventTrigger("Wait", Commands.waitSeconds(5).withName("WAITING"));
     
  }
}
