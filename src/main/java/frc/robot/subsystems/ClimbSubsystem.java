import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class ClimbSubsystem extends SubsystemBase {
  public final int CLIMB_MOTOR_ID = 0; //placeholder

  private final TalonFX climbMotor;

  private double targetPos;
  public final double TARGET_POS_START = 0.0;
  public final double TARGET_POS_END = 0.0;

  //private final FlywheelSim motorSim; //add sim class later

  public ClimbSubsystem() {
    climbMotor = new TalonFX(CLIMB_MOTOR_ID);
    climbConfig();
    if (RobotBase.isSimulation()) {
      //motorSim = null; //add sim later
    } else {
      //motorSim = null;
    }
  }

  public void climbConfig() {

    TalonFXConfigurator cfg = climbMotor.getConfigurator();
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // enabling current limits
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = 50;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 0;
    talonFXConfiguration.Slot0.kA = 0.5;
    talonFXConfiguration.Slot0.kS = 2.2;
    talonFXConfiguration.Slot0.kP = 10;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = climbMotor.getConfigurator().apply(talonFXConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      DriverStation.reportError("Failed to apply ClimbMotor configs: " + status, false);
    }
  }
  
  public Command setPos(double pos) {
    return runOnce(
            () -> {
              climbMotor.setPosition(pos);
              targetPos = pos;
            });
  }

  public void stopMotor() {
    climbMotor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    // motorSim.setInput(climbMotor.getSimState().getMotorVoltage());
    // motorSim.update(TimedRobot.kDefaultPeriod); // every 20 ms
  }

  @Override
  public void periodic() {
    //add stuff later
  }
}
