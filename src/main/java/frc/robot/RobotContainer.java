/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Autos.AutoName;
import frc.robot.commands.ChassisDriveManual;
import frc.robot.commands.ChassisVisionTargeting;
import frc.robot.commands.ChassisVisionTargetingFancy;
import frc.robot.commands.ClimbManual;
import frc.robot.commands.IntakeDisengage;
import frc.robot.commands.IntakeEngage;
import frc.robot.commands.PassthroughEject;
import frc.robot.commands.PassthroughIdle;
import frc.robot.commands.ShooterSetRPM;
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

  public Chassis chassis = new Chassis();
  public final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Passthrough passthrough = new Passthrough();
  private final Shooter shooter = new Shooter();
  private final Spinner spinner = new Spinner();
  private final Vision vision = new Vision(navX);

  private final Autos autos = new Autos(navX, shooter, intake, vision, passthrough, chassis);
  

  // private final ExampleCommand autoCommand = new ExampleCommand(exampleSubsystem);
  private Command autoCommand;


  //Inputs
  Joystick driver = new Joystick(0);
  JoystickButton visionAimToTarget = new JoystickButton(driver, 5);
  JoystickButton visionAimToTargetFancy = new JoystickButton(driver, 7);
  JoystickButton shiftButton = new JoystickButton(driver, 6);
  JoystickButton fineTurning = new JoystickButton(driver, 8);

  Joystick controller = new Joystick(1);
  Button intakeButton = new JoystickButton(controller, 1);
  Button shooterSpinCalculatedSpeed = new JoystickButton(controller, 3);//TODO: Implement properly
  Button shooterSpinDefaultSpeed = new JoystickButton(controller, 2);
  JoystickButton loadBallManually = new JoystickButton(controller, 4);//backup button: Doesn't need to be used much
  JoystickButton shoot = new JoystickButton(controller, 5);
  JoystickButton eject = new JoystickButton(controller, 6);

  Button climbEnable = new JoystickButton(controller, 7);
  Button climbHookRetract = new JoystickButton(controller, 8);
  Button climbHookGrab = new JoystickButton(controller,9);
  Button climbTranslateRight = new JoystickButton(controller, 10);
  Button climbTranslateSideways = new JoystickButton(controller, 11);

  //DEBUG: Will need to be removed soon
  //Button tempClimbSpoolPositive = new JoystickButton(controller, 12);
  // Button tempClimbSpoolNegative = new JoystickButton(controller, 13);

  /*
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    navX.reset();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    shiftButton.whenPressed(new InstantCommand(()->chassis.shift(Gear.HIGH)));
    shiftButton.whenReleased(new InstantCommand(()->chassis.shift(Gear.LOW)));
    visionAimToTarget.whileHeld(new ChassisVisionTargeting(vision, navX, chassis));
    visionAimToTargetFancy.whileHeld(new ChassisVisionTargetingFancy(vision, navX, chassis));
    ;
    fineTurning.whileHeld(new ChassisDriveManual(() -> driver.getRawAxis(1), () -> 0.5 * driver.getRawAxis(2), chassis));

    /* Player 2 normal Buttons */
    intakeButton.whenPressed(new IntakeEngage(intake));
    intakeButton.whenReleased(new IntakeDisengage(intake).withTimeout(0.1));

    shooterSpinDefaultSpeed.whenPressed(()->passthrough.prepareForShooting());
    shooterSpinDefaultSpeed.whileHeld(new ConditionalCommand(
      new ShooterSetRPM( ()->Constants.distanceToRPM.getOutputAt(20*12), shooter),
      new InstantCommand( ()->{} ),
      ()->{return shooter.isOnTarget();}
    ));
    shooterSpinDefaultSpeed.whileHeld(new ShooterSetRPM(()->Constants.distanceToRPM.getOutputAt(20*12), shooter));
    // shooterSpinDefaultSpeed.whileHeld(new ShooterSetRPM(()->SmartDashboard.getNumber("shooter/RMPDebugSet", 1000), shooter));
    shooterSpinDefaultSpeed.whenReleased(()->passthrough.prepareForLoading());

    shooterSpinCalculatedSpeed.whileHeld(new ShooterSetRPM(()->{
      if( vision.isTargetValid() ){ return vision.getRPMForDistance(vision.getDistance()); } 
      else { return 1000;}
    }, shooter));
    
    shoot.whenPressed(()->passthrough.shoot());
    loadBallManually.whenPressed(new InstantCommand(()->passthrough.loadBall(),passthrough));
    eject.whenPressed(new PassthroughEject(passthrough, intake));

    /*Player 2 Climb Buttons */

    climbHookRetract.whenPressed(()->{
      if(!climbEnable.get())return;
      climber.setHookAngle(0);
    });
    climbHookGrab.whenPressed(()->{
      if(!climbEnable.get())return;
      climber.setHookAngle(180);
    });
    //TODO: Not implemented in hardware yet
    // translationMoveForwards.whenPressed(new ClimberSetTranslation(()->0.2, climber));
    // translationMoveBackwards.whenPressed(new ClimberSetTranslation(()->-0.2,climber));

    /*Debug Buttons */ //TODO: Remove these before competitions
    // tempClimbSpoolPositive.whileHeld(new SpinSpoolPositive(climber));
    // tempClimbSpoolNegative.whileHeld(new SpinSpoolNegative(climber));
    configAutoSelector();
  }

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<Boolean> limelightInAuto = new SendableChooser<>();

  public void configAutoSelector(){
    autoChooser.setDefaultOption("Basic", new WaitCommand(0));
    autoChooser.setDefaultOption("LeftMostPos", new WaitCommand(0));
    autoChooser.addOption("LeftRendevous", new WaitCommand(0));
    autoChooser.setDefaultOption("RightRendevous", new WaitCommand(0));
    autoChooser.addOption("RightMostPos", new WaitCommand(0));
    SmartDashboard.putData("autos/position", autoChooser);
    
    limelightInAuto.setDefaultOption("No", false);
    limelightInAuto.addOption("Yes", true);
    SmartDashboard.putData("autos/limelight", limelightInAuto);
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
      new ClimbManual(()->climbEnable.get(),()->controller.getRawAxis(3),climber)
    );

    //Ensure that after vision is used, it returns to Driver view
    vision.setDefaultCommand( new FunctionalCommand(
      ()->vision.driverPipeline(),
      ()->{}, 
      (interrupted)->{}, 
      ()->false,
      vision
    ));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    AutoName chosenAuto = AutoName.FULL_NEAR_TRENCH; // do the smartDashboard stuff instead

    switch(chosenAuto) {
    case SAFETY:
      return autos.getSafety();

    case BASIC_FAR_TRENCH:
      // return autos.getBasicFarTrench();

    case BASIC_NEAR_TRENCH:
      return autos.getBasicNearTrench();

    case BASIC_PORT_CENTERED:
      // return autos.getBasicPortCentered();

    case BASIC_RONDE_CENTERED:
      // return autos.getBasicRondeCentered();

    case FULL_NEAR_TRENCH:
      return autos.trench3BallAuto();
    
    case FULL_CENTERED_RONDE:
      return autos.ronde3BallAuto();

    default: // just do safety
      return autos.getSafety();
    }

  }

  // public Command turn(DoubleSupplier targetAngle) {
  //   return new ChassisDriveToHeadingBasic(0, targetAngle, 3 /*Degrees*/, 0.05 /*Meters*/, navX, chassis);
  // }

  // public Command driveForward(double driveDistance) {
  //   return new ChassisDriveToHeadingBasic(driveDistance, () -> 0, 3 /*Degrees*/, 0.05 /*Meters*/, navX, chassis);
  // }

  // public double calculateAngleFromHome() {
  //   return calculateTargetAngle(navX.getCompassHeading(), Constants.INITIAL_COMPASS_HEADING);
  // }

  // public double calculateTargetAngle(double startAngle, double finalAngle) {
  //   double angleDifference = finalAngle - startAngle;
  //   if(angleDifference <= 0) return 360 + angleDifference;
  //   if(angleDifference <= 180) return angleDifference;
  //   else return 360 - angleDifference;
  // }


  // public double calculateAngleToInitialCompassBearing() {

    //Valid basically as long as we never reset the navx's gyro
    // return 0 - navX.getAngle();

    /* TODO: We may or may not need this code, leave it in here for now
    //Does not work if the gyro is un-calibrated.
    double initialAngle = navX.getCompassHeading();
    double finalAngle = Constants.INITIAL_COMPASS_HEADING;
    double angleDifference = finalAngle - initialAngle;
    SmartDashboard.putNumber("chassis/Pre Modification AngleDifference", angleDifference);
    */    
  // }



  


}
