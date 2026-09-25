package frc.robot;

import java.util.function.DoubleSupplier;

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

public class Flywheels extends SubsystemBase {
  private final TalonFX flywheelOne;
  private final TalonFX flywheelTwo;

  public static double speed = 1; // placeholder

  // Configs: PIDS and limits
  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;
  double kA = 0.0;
  double kV = 0.0;
  double kS = 0.0;
  double kG = 0.0;

  private static final int supplyCurrentLimit = 20;
  private static final int statorCurrentLimit = 40;
  private static final boolean supplyCurrentLimitEnable = true;
  private static final boolean statorCurrentLimitEnable = true;
  private static final int motionMagicAcceleration = 40;
  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
  private final Follower follow =
      new Follower(Hardware.FLYWHEEL_ONE_ID, MotorAlignmentValue.Opposed);
  public final double FLYWHEEL_TOLERANCE = 5; // TODO: Tune tolerance as well

  public Flywheels() {
    flywheelOne = new TalonFX(Hardware.FLYWHEEL_ONE_ID);
    flywheelTwo = new TalonFX(Hardware.FLYWHEEL_TWO_ID);

    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // TODO: tune configs and PIDS, currently placeholder
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = supplyCurrentLimitEnable;
    config.CurrentLimits.StatorCurrentLimitEnable = statorCurrentLimitEnable;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kA = kA;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;
    config.Slot0.kG = kG;

    config.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration; // RPS^2

    flywheelOne.getConfigurator().apply(config);
    flywheelTwo.getConfigurator().apply(config);
  }

  public Command runFlywheels() {
    return startEnd(
            () -> {
              setFlywheelVelocity(speed);
            },
            () -> {
              stopCommand();
            })
        .withName("Run flywheels");
  }

  private void setFlywheelVelocity(double rps) {
    request.Velocity = rps;
    flywheelOne.setControl(request);
    flywheelTwo.setControl(follow);
  }

  public Command suppliedSetVelocity(DoubleSupplier rps) {
    return startEnd(
            () -> {
              request.Velocity = rps.getAsDouble();
              flywheelOne.setControl(request);
              flywheelTwo.setControl(follow);
            },
            () -> {
              flywheelOne.stopMotor();
              flywheelTwo.stopMotor();
            })
        .withName("Supplied velocity command");
  }

  public boolean isAtSpeed() {
    return Math.abs(flywheelOne.getVelocity().getValueAsDouble() - request.Velocity)
        <= FLYWHEEL_TOLERANCE;
  }

  private void stopCommand() {
    flywheelOne.stopMotor();
  }
}
