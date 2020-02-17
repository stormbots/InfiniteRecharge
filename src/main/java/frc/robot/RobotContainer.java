/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChassisDriveManual;
import frc.robot.commands.ChassisDrivePDControl;
import frc.robot.commands.ChassisDriveToHeadingBasic;
import frc.robot.commands.ChassisVisionTargeting;
import frc.robot.commands.DisengageIntake;
import frc.robot.commands.EngageIntake;
import frc.robot.commands.PassthroughIdle;
import frc.robot.commands.PassthroughLoadManually;
import frc.robot.commands.RunShooter;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChassisDriveManual;
import frc.robot.commands.ClimbManual;
import frc.robot.commands.ClimberSetHookRotation;
import frc.robot.commands.ClimberSetTranslation;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SpinSpoolNegitive;
import frc.robot.commands.SpinSpoolPositive;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.Gear;
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

  // private final ADIS16448_IMU gyro = new ADIS16448_IMU();
  public final AHRS navX = new AHRS(SPI.Port.kMXP);

  private final Chassis chassis = new Chassis();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Passthrough passthrough = new Passthrough();
  private final Shooter shooter = new Shooter();
  private final Spinner spinner = new Spinner();
  private final Vision vision = new Vision(navX);
  

  // private final ExampleCommand autoCommand = new ExampleCommand(exampleSubsystem);
  private Command autoCommand;


  //Inputs
  Joystick driver = new Joystick(0);
  JoystickButton visionButton = new JoystickButton(driver, 5);
  JoystickButton shiftButton = new JoystickButton(driver, 6);

  Joystick controller = new Joystick(1);
  Button intakeButton = new JoystickButton(controller, 1);
  Button shooterButton = new JoystickButton(controller, 2);

  private JoystickButton loadBall = new JoystickButton(driver, 1);
  private JoystickButton prepareForLoading = new JoystickButton(driver, 2);
  private JoystickButton prepareForShooting = new JoystickButton(driver, 3);
  private JoystickButton shoot = new JoystickButton(driver, 4);


  Button armMoveUp = new JoystickButton(driver, 6);
  Button climbHookReseat = new JoystickButton(driver, 7);
  Button translationMoveForwards = new JoystickButton(driver, 8);
  Button climbHookGrab = new JoystickButton(driver,9);
  Button translationMoveBackwards = new JoystickButton(driver, 10);
  Button armMoveDown = new JoystickButton(driver, 11);
  Button climbButton = new JoystickButton(driver, 1);
  Button climbEnable = new JoystickButton(driver, 4);
  Button tempButtonPositive = new JoystickButton(driver, 2);
  Button tempButtonNegative = new JoystickButton(driver, 4);

  /*
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
    visionButton.whileHeld(new ChassisVisionTargeting(vision, navX, chassis));
    shiftButton.whenPressed(new InstantCommand(()->chassis.shift(Gear.HIGH)));
    shiftButton.whenReleased(new InstantCommand(()->chassis.shift(Gear.LOW)));

    intakeButton.whenPressed(new EngageIntake(intake));
    intakeButton.whenReleased(new DisengageIntake(intake).withTimeout(0.1));
    //alternate toggle version
    //intakeButton.toggleWhenPressed(engage.andThen(disengage.withTimeout(0.1))); 
    shooterButton.whenPressed(new RunShooter(()->1000, shooter));

    
    loadBall.whenPressed(new PassthroughLoadManually(passthrough));

    shoot.whenPressed(new InstantCommand(()->{
      passthrough.eject(); 
    }));

    climbHookReseat.whenPressed(()->{
      if(!climbEnable.get())return;
      climber.setHookAngle(0);
    });
    climbHookGrab.whenPressed(()->{
      if(!climbEnable.get())return;
      climber.setHookAngle(180);
    });
    // translationMoveForwards.whenPressed(new ClimberSetTranslation(()->0.2, climber));
    // translationMoveBackwards.whenPressed(new ClimberSetTranslation(()->-0.2,climber));
    tempButtonPositive.whileHeld(new SpinSpoolPositive(climber));
    tempButtonNegative.whileHeld(new SpinSpoolNegitive(climber));


    //climbButton.whileHeld(new ClimbUp(climber));
  }

  /** 
   * Set up default commands
   */
  private void configureDefaultCommands(){

    //Do any joystick math in the command itself, not here. 
    chassis.setDefaultCommand(
      new ChassisDriveManual(()->driver.getRawAxis(1),  ()->driver.getRawAxis(2), chassis)
      );

    passthrough.setDefaultCommand(
      new PassthroughIdle(passthrough)
    );

    //TODO This should also only activate near end of a match eventually
    climber.setDefaultCommand(
      new ClimbManual(()->climbEnable.get(),()->driver.getRawAxis(3),climber)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // autoCommand = new ChassisDriveToHeadingBasic(1, 180, navX, chassis);


    Command turnToShoot = new SequentialCommandGroup(
      //vision.visionStuff();
      turn(() -> -45)
    );
    

    Command turnAwayFromShooting = new SequentialCommandGroup(
      turn(() -> 45),//calculateDistnaceToHome() ),
      driveForward(-2)
    );

    autoCommand = new SequentialCommandGroup(
      turnToShoot,
      turnAwayFromShooting
    );


    // An ExampleCommand will run in autonomous
    return autoCommand;
  }

  public Command turn(DoubleSupplier targetAngle) {
    return new ChassisDriveToHeadingBasic(0, targetAngle, 3 /*Degrees*/, 0.05 /*Meters*/, navX, chassis);
  }

  public Command driveForward(double driveDistance) {
    return new ChassisDriveToHeadingBasic(driveDistance, () -> 0, 3 /*Degrees*/, 0.05 /*Meters*/, navX, chassis);
  }

  // public double calculateAngleFromHome() {
  //   return calculateTargetAngle(navX.getCompassHeading(), Constants.INITIAL_COMPASS_HEADING);
  // }

  // public double calculateTargetAngle(double startAngle, double finalAngle) {
  //   double angleDifference = finalAngle - startAngle;
  //   if(angleDifference <= 0) return 360 + angleDifference;
  //   if(angleDifference <= 180) return angleDifference;
  //   else return 360 - angleDifference;
  // }


  public double calculateDistnaceToHome() {

    return 0 - navX.getAngle();

    // double initialAngle = navX.getCompassHeading();
    // double finalAngle = Constants.INITIAL_COMPASS_HEADING;

    // double angleDifference = finalAngle - initialAngle;

    // SmartDashboard.putNumber("Chassis/Pre Modificaiton AngleDifference", angleDifference);


    // if(angleDifference > 180) angleDifference += -360;
    // if(angleDifference < -180) angleDifference += 360;

    // SmartDashboard.putNumber("Chassis/Post Modificaiton AngleDifference", angleDifference);

    // return angleDifference;

  }



  


}
