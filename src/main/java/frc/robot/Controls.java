// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.util.GetTargetFromPose;
import frc.robot.util.tuning.WheelRadiusCharacterization;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Controls {

  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final int TEST_CONTROLLER_PORT = 1;
  private static final double JOYSTICK_DEADBAND = 0.1;
  private final Subsystems s;
  private static final double SWERVE_DEADBAND = 0.001;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  private final CommandXboxController testController =
      new CommandXboxController(TEST_CONTROLLER_PORT);

  public static final double MaxSpeed = AlphaTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  public Controls(Subsystems subsystems) {
    s = subsystems;
    configureIntake();
    // Configure the trigger bindings
    configureFlywheelBindings();
    configureDrivebaseBindings();
  }

  // takes the X value from the joystick, and applies a deadband and input scaling
  private double getDriveX() {
    // Joystick +Y is back
    // Robot +X is forward
    double input = MathUtil.applyDeadband(-driverController.getLeftY(), JOYSTICK_DEADBAND);
    return input * MaxSpeed;
  }

  private Trigger connected(CommandXboxController controller) {
    return new Trigger(() -> controller.isConnected());
  }

  // takes the Y value from the joystick, and applies a deadband and input scaling
  private double getDriveY() {
    // Joystick +X is right
    // Robot +Y is left
    double input = MathUtil.applyDeadband(-driverController.getLeftX(), JOYSTICK_DEADBAND);
    return input * MaxSpeed;
  }

  // takes the rotation value from the joystick, and applies a deadband and input
  // scaling
  private double getDriveRotate() {
    // Joystick +X is right
    // Robot +angle is CCW (left)
    double input = MathUtil.applyDeadband(-driverController.getRightX(), JOYSTICK_DEADBAND);
    return input * MaxSpeed;
  }

  private void configureFlywheelBindings() {
    if (s.flywheels == null) {
      return;
    } else {
      driverController.rightTrigger().whileTrue(s.flywheels.runFlywheels());
    }
  }

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(SWERVE_DEADBAND)
          .withRotationalDeadband(SWERVE_DEADBAND)
          .withDriveRequestType(DriveRequestType.Velocity);

  private void configureIntake() {
    if (s.intakeSubsystem != null) {
      driverController.rightBumper().whileTrue(s.intakeSubsystem.startIntake());
    }
  }

  private void configureDrivebaseBindings() {
    if (s.drivebaseSubsystem == null) {
      // Stop running this method
      return;
    }

    // readyToShoot = GetTargetFromPose.autoShoot(s.drivebaseSubsystem);

    connected(testController)
        .and(testController.y())
        .whileTrue(
            WheelRadiusCharacterization.wheelRadiusCharacterizationCommand(s.drivebaseSubsystem));
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    // the driving command for just driving around
    s.drivebaseSubsystem.setDefaultCommand(
        // s.drivebaseSubsystem will execute this command periodically

        // applying the request to drive with the inputs
        s.drivebaseSubsystem
            .applyRequest(
                () ->
                    drive
                        .withVelocityX(getDriveX())
                        .withVelocityY(getDriveY())
                        .withRotationalRate(getDriveRotate()))
            .withName("Drive"));

    driverController
        .a()
        .whileTrue(Commands.run(() -> s.drivebaseSubsystem.setControl(brake)).withName("Brake"));

    // reset pose incase vision is bugging
    driverController
        .back()
        .onTrue(
            s.drivebaseSubsystem
                .runOnce(() -> s.drivebaseSubsystem.resetPose(GetTargetFromPose.getRestPose()))
                .withName("Reset to Hub"));
  }
}
