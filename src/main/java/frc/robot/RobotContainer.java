/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChassisDriveManual;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.ClimberSetHookRotation;
import frc.robot.commands.ClimberSetPosition;
import frc.robot.commands.ClimberSetTranslation;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  private final Chassis chassis = new Chassis();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Passthrough passthrough = new Passthrough();
  private final Shooter shooter = new Shooter();
  private final Spinner spinner = new Spinner();
  private final Vision vision = new Vision();

  private final ExampleCommand autoCommand = new ExampleCommand(exampleSubsystem);

  //Inputs
  Joystick driver = new Joystick(0);
  Button armMoveUp = new JoystickButton(driver, 6);
  Button climbHookReseat = new JoystickButton(driver, 7);
  Button translationMoveForwards = new JoystickButton(driver, 8);
  Button climbHookGrab = new JoystickButton(driver,9);
  Button translationMoveBackwards = new JoystickButton(driver, 10);
  Button armMoveDown = new JoystickButton(driver, 11);
  Button climbButton = new JoystickButton(driver, 1);
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    climbHookReseat.whenPressed(new ClimberSetHookRotation(()->0,climber));
    climbHookGrab.whenPressed(new ClimberSetHookRotation(()->180,climber));
    translationMoveForwards.whenPressed(new ClimberSetTranslation(()->0.2, climber));
    translationMoveBackwards.whenPressed(new ClimberSetTranslation(()->-0.2,climber));

    climbButton.whileHeld(new ClimbUp(climber));
  }

  /** 
   * Set up default commands
   */
  private void configureDefaultCommands(){

    //Do any joystick math in the command itself, not here. 
    chassis.setDefaultCommand(
      new ChassisDriveManual(()->driver.getRawAxis(1),  ()->driver.getRawAxis(4),chassis)
      );
    //TODO! We want this on a button when held
    //TODO This should also only activate near end of a match eventually
    climber.setDefaultCommand(
    //  DEBUG JOYSTICK STUFF
    //  new ClimberSetPosition(()->Lerp.lerp(driver.getRawAxis(3), -1, 1, 0, 90),climber)
     new ClimberSetPosition(()->driver.getRawAxis(3)*70,climber)
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}
