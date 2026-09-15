package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Flywheels extends SubsystemBase {
  private final TalonFX FlywheelOne;
  private final TalonFX FlywheelTwo;

  public static double speed = 1; // placeholder

  // Configs: PIDS and limits
  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;
  double kA = 0.0;
  double kV = 0.0;
  double kS = 0.0;
  double kG = 0.0;

  int SupplyCurrentLimit = 20;
  int StatorCurrentLimit = 40;
  boolean SupplyCurrentLimitEnable = true;
  boolean StatorCurrentLimitEnable = true;
  int MotionMagicAcceleration = 40;

  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
  private final Follower follow =
      new Follower(Hardware.FLYWHEEL_ONE_ID, MotorAlignmentValue.Opposed);
  public final double FLYWHEEL_TOLERANCE = 5; // TODO: Tune tolerance as well

  public Flywheels() {
    FlywheelOne = new TalonFX(Hardware.FLYWHEEL_ONE_ID);
    FlywheelTwo = new TalonFX(Hardware.FLYWHEEL_TWO_ID);

    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfigurator flConfigurator = FlywheelOne.getConfigurator();

    // TODO: tune configs and PIDS, currently placeholder
    config.CurrentLimits.SupplyCurrentLimit = SupplyCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = StatorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = SupplyCurrentLimitEnable;
    config.CurrentLimits.StatorCurrentLimitEnable = StatorCurrentLimitEnable;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kA = kA;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;

    config.MotionMagic.MotionMagicAcceleration = MotionMagicAcceleration; // RPS^2

    flConfigurator.apply(config);
  }

  public Command setFlywheelVelocity(double rps) {
    return runEnd(
            () -> {
              request.Velocity = rps;
              FlywheelOne.setControl(request);
              FlywheelTwo.setControl(follow);
            },
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Set Velocity");
  }
  public void setVelocityCommand() {
    setFlywheelVelocity(speed);
  }

  public Command suppliedSetVelocity(DoubleSupplier rps) {
    return runEnd(
            () -> {
              request.Velocity = rps.getAsDouble();
              FlywheelOne.setControl(request);
              FlywheelTwo.setControl(follow);
            },
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Supplied velocity command");
  }

  public void setVelocityRPS(double rps) {
    request.Velocity = rps;
    FlywheelOne.setControl(request);
    FlywheelTwo.setControl(follow);
  }

  public Command stopCommand() {
    return runOnce(
            () -> {
              FlywheelOne.stopMotor();
              FlywheelTwo.stopMotor();
            })
        .withName("Stop motors command");
  }
}
