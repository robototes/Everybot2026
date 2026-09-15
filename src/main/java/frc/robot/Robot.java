// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.SubsystemConstants;
import frc.robot.subsystems.auto.AutoBuilderConfig;
import frc.robot.subsystems.auto.AutoLogic;
import frc.robot.subsystems.auto.AutonomousField;
import frc.robot.util.AllianceUtils;
import frc.robot.util.BuildInfo;
import frc.robot.util.DriveStateNtLogger;
import frc.robot.util.DriveStateSignalLogger;
import frc.robot.util.GCMonitor;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.simulation.RobotSim;
import frc.robot.util.tuning.LauncherConstants;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private final Controls controls;
  public final Subsystems subsystems;
  private final PowerDistribution PDH;
  private final RobotSim robotSim;
  private final Mechanism2d mechanismRobot;
  private static final double BROWNOUT_VOLTAGE = 7.0;
  private static final double DATA_LOG_FLUSH_PERIOD_S = 1.0 / 14.0; // 14 Hz flush
  private final DriveStateNtLogger driveBaseSim;
  private final DriveStateSignalLogger logger;

  // Cached time for robot.periodic()
  private double LAST_TIME = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  protected Robot() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    // logging
    if (RobotBase.isReal()) {
      DataLogManager.start("", "", DATA_LOG_FLUSH_PERIOD_S);
      DriverStation.startDataLog(DataLogManager.getLog(), true);
    }
    PDH = new PowerDistribution(Hardware.PDH_ID, PowerDistribution.ModuleType.kRev);
    LiveWindow.disableAllTelemetry();
    LiveWindow.enableTelemetry(PDH);
    BuildInfo.logBuildInfo();
    // Start GC monitor to count garbage collections and publish to SmartDashboard
    frc.robot.util.GCMonitor.start();

    // Set brownout Voltage
    RobotController.setBrownoutVoltage(BROWNOUT_VOLTAGE);

    // Loads the field layout before auto to prevent any delay
    AllianceUtils.getHubTranslation2d();
    mechanismRobot = new Mechanism2d(Units.inchesToMeters(30), Units.inchesToMeters(24));
    SmartDashboard.putData("Mechanism2d", mechanismRobot);
    subsystems = new Subsystems(mechanismRobot);

    controls = new Controls(subsystems);

    if (DRIVEBASE_ENABLED) {
      AutoBuilderConfig.buildAuto(subsystems.drivebaseSubsystem, false);
    }
    AutoLogic.init(subsystems);
    if (Robot.isSimulation()) {
      robotSim = new RobotSim(subsystems.drivebaseSubsystem);
    } else {
      robotSim = null;
    }
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> DataLogManager.log("Command initialized: " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (command, interruptor) ->
                DataLogManager.log(
                    "Command interrupted: "
                        + command.getName()
                        + "; Cause: "
                        + interruptor.map(cmd -> cmd.getName()).orElse("<none>")));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> DataLogManager.log("Command finished: " + command.getName()));

    SmartDashboard.putData(CommandScheduler.getInstance());

    if (SubsystemConstants.DRIVEBASE_ENABLED) {
      AutoLogic.initCommandsAndPaths(false);
      AutonomousField.initSmartDashBoard(() -> "Field", 0, 0, this::addPeriodic);

      AutoLogic.initSmartDashBoard();
      CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    logger = new DriveStateSignalLogger();
    subsystems.drivebaseSubsystem.registerTelemetry(logger::telemeterize);
    driveBaseSim = logger.DrivebaseSim(Controls.MaxSpeed);

    // Explicitly register struct schemas with the DataLog
    if (RobotBase.isReal()) {
      DataLog log = DataLogManager.getLog();
      log.addSchema(Pose2d.struct);
      log.addSchema(Pose3d.struct);
      log.addSchema(ChassisSpeeds.struct);
      log.addSchema(SwerveModuleState.struct);
      log.addSchema(SwerveModulePosition.struct);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Resume logging every X seconds
    double time = Timer.getFPGATimestamp();
    if (time - LAST_TIME >= 1) {
      LAST_TIME = time;
      DataLogManager.getLog().resume();
    }

    // var robotState = subsystems.drivebaseSubsystem.getState();
    // LauncherConstants.update(robotState.Pose, subsystems.drivebaseSubsystem);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    driveBaseSim.update();
    LauncherConstants.UpdateNT(subsystems.drivebaseSubsystem.getState().Pose);

    SmartDashboard.putNumber("GCCount", GCMonitor.getGcCount());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();

    HubShiftUtil.initialize();
  }

  @Override
  public void teleopPeriodic() {}

  /** This function is called once when teleop mode is exited. */
  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

    robotSim.updateFuelSim();
  }
}
