package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Hardware;

// for intake with 2 motors again

public class ClaudeTake {
  private final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0.0).withEnableFOC(true);

  private final TalonFX intakeFive;
  private final TalonFX intakeSix;
  private static final double TARGET_DUTYCYCLE = 1.0;

  private final Follower followRequest =
      new Follower(Hardware.IntakeThree, MotorAlignmentValue.Aligned);

  public ClaudeTake() {
    intakeFive = new TalonFX(Hardware.IntakeFive);
    intakeSix = new TalonFX(Hardware.IntakeSix);
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
    intakeFive.getConfigurator().apply(talonFXConfigs);
    intakeSix.getConfigurator().apply(talonFXConfigs);
  }

  public Command startIntake() {
    return Commands.startEnd(
            () -> {
              runIntake(TARGET_DUTYCYCLE);
            },
            () -> {
              stopIntake();
            })
        .withName("Run Intake");
  }

  private void runIntake(double dutycycle) {
    intakeFive.setControl(intakeDutyCycle.withOutput(dutycycle));
    intakeSix.setControl(followRequest);
  }

  private void stopIntake() {
    intakeFive.stopMotor();
    intakeSix.stopMotor();
  }
}
