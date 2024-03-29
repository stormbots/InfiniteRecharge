/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.BotName;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robot;

  Compressor compressor = new Compressor();
  PowerDistributionPanel pdp = new PowerDistributionPanel();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Figure out which bot we're on.
    String botString = Preferences.getInstance().getString("botName", "none").toUpperCase().trim();
    switch(botString){
      case "COMP":     Constants.botName = BotName.COMP;break;
      case "PRACTICE": Constants.botName = BotName.PRACTICE;break;
      case "TABI":     Constants.botName = BotName.TABI;break;
      default:
        botString = "COMP";
        Constants.botName = BotName.COMP;
        System.err.println("ROBOT NAME NOT DEFINED:");
        System.err.println("Assuming COMP for safety: View Preferences to change");
    }
    Preferences.getInstance().putString("botName",botString);
    SmartDashboard.setPersistent("Preferences/botName");
    //make sure any robot-specific constants are handled properly.
    Constants.Initialize();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robot = new RobotContainer();

    //TODO: move to robotContainer constructor
    Constants.INITIAL_COMPASS_HEADING = robot.navX.getCompassHeading();

    //Option to disable default sensor/motor spam on NetworkTables
    //LiveWindow.disableAllTelemetry();

    SmartDashboard.putData(new PowerDistributionPanel());
    SmartDashboard.putData(robot.navX);

    // compressor.clearAllPCMStickyFaults();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //SmartDashboard.putNumber("compressor/amps", pdp.);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    robot.chassis.setMotorIdleModes(IdleMode.kCoast);
  }

  @Override
  public void disabledPeriodic() {

    SmartDashboard.putNumber("Chassis/COMPASS HEADING", robot.navX.getCompassHeading());
    SmartDashboard.putBoolean("compressor/CurrentTooHight",compressor.getCompressorCurrentTooHighFault());
    SmartDashboard.putBoolean("compressor/CurrentShorted",compressor.getCompressorShortedFault());
    SmartDashboard.putBoolean("compressor/CurrentNotConnected",compressor.getCompressorNotConnectedFault());
    SmartDashboard.putBoolean("compressor/CurrentShortedSticky",compressor.getCompressorShortedStickyFault());
    SmartDashboard.putBoolean("compressor/CurrentNotConnectedSticky",compressor.getCompressorNotConnectedStickyFault());
    SmartDashboard.putBoolean("compressor/CurrentTooHightSticky",compressor.getCompressorCurrentTooHighStickyFault());

    SmartDashboard.putData(robot.navX);

    robot.vision.driverPipeline();
    

  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    robot.climber.resetEncoders();
    
    Constants.INITIAL_COMPASS_HEADING = robot.navX.getCompassHeading();  
    robot.chassis.setMotorIdleModes(IdleMode.kBrake);

    robot.navX.reset();

    autonomousCommand = robot.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    SmartDashboard.putNumber("Chassis/compass error", robot.navX.getCompassHeading() - Constants.INITIAL_COMPASS_HEADING);

  }

  @Override
  public void teleopInit() {
    robot.chassis.setMotorIdleModes(IdleMode.kBrake);

    robot.climber.resetEncoders();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // robot.navX.reset();
    robot.climber.setMotorIdleModes(IdleMode.kCoast);
    robot.climber.setHeight(robot.climber.CLIMBER_BASE_HEIGHT);
    robot.climber.enable = false;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
