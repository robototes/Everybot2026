package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.FLYWHEELS_ENABLED;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class Subsystems {

  // <SUBSYSTEM>_ENABLED constants go here
  public static class SubsystemConstants {
    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean FLYWHEELS_ENABLED = true;
  }

  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final Flywheels Flywheels;

  public Subsystems(Mechanism2d mechanism2d) {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?

    // Pattern is

    // if (SUBSYSTEM_ENABLED){
    // subsystem = new SubsystemName();
    // } else {
    // subsystem = null;
    // }

    if (DRIVEBASE_ENABLED) {
      drivebaseSubsystem = AlphaTunerConstants.createDrivetrain();

    } else {
      drivebaseSubsystem = null;
    }
    if (FLYWHEELS_ENABLED) {
      Flywheels = new Flywheels();
    } else {
      Flywheels = null;
    }
  }
}
