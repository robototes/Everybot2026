package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class IntakeSubsystem extends SubsystemBase {
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

  private final TalonFX intakeMotor;

  // TODO: set rps
  private static final double TARGET_RPS = 0.0;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Hardware.IntakeMotor);
    motorConfigs();
  }

  private void motorConfigs() {
    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast; // KEEP THIS AT COAST
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // TODO: set values
    talonFXConfigs.Slot0.kP = 0.0;
    talonFXConfigs.Slot0.kS = 0.0;
    talonFXConfigs.Slot0.kA = 0.0;

    // configurator
    intakeMotor.getConfigurator().apply(talonFXConfigs);
  }

  public Command startIntake() {
    return startEnd(
            () -> {
              runIntake(TARGET_RPS);
            },
            () -> {
              stopIntake();
            })
        .withName("Run Intake");
  }

  private void runIntake(double velocity) {
    intakeMotor.setControl(velocityRequest.withVelocity(velocity));
  }

  private void stopIntake() {
    intakeMotor.stopMotor();
  }
}
