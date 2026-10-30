package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.FLYWHEELS_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INTAKEV2_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INTAKEV3_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeV2;
import frc.robot.subsystems.intake.IntakeV3;

public class Subsystems {

  // <SUBSYSTEM>_ENABLED constants go here
  public static class SubsystemConstants {
    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean INTAKE_ENABLED = true;
    public static final boolean FLYWHEELS_ENABLED = true;
    public static final boolean INTAKEV2_ENABLED = true;
    public static final boolean INTAKEV3_ENABLED = true;
  }

  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public final Flywheels flywheels;
  public final IntakeV2 intakeV2;
  public final IntakeV3 intakeV3;

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

    if (INTAKE_ENABLED) {
      intakeSubsystem = new IntakeSubsystem();
    } else {
      intakeSubsystem = null;
    }
    if (FLYWHEELS_ENABLED) {
      flywheels = new Flywheels();
    } else {
      flywheels = null;
    }
    if (INTAKEV2_ENABLED) {
      intakeV2 = new IntakeV2();
    } else {
      intakeV2 = null;
    }
    if (INTAKEV3_ENABLED) {
      intakeV3 = new IntakeV3();
    } else {
      intakeV3 = null;
    }
  }
}
