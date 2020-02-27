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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChassisDriveManual;
import frc.robot.commands.ChassisDriveToHeadingBasic;
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
    shooterSpinDefaultSpeed.whileHeld( new ShooterSetRPM( ()->Constants.distanceToRPM.getOutputAt(20*12), shooter) );
    // shooterSpinDefaultSpeed.whileHeld(new ShooterSetRPM(()->SmartDashboard.getNumber("shooter/RMPDebugSet", 1000), shooter));
    shooterSpinDefaultSpeed.whenReleased(()->passthrough.prepareForLoading());

    shooterSpinCalculatedSpeed.whileHeld(new ShooterSetRPM(()->{
      if( vision.isTargetValid() ){ return vision.getRPMForDistance(vision.getDistance()); } 
      else { return 1000;}
    }, shooter));
    
    //conditional
    shoot.whileHeld(new ConditionalCommand(
      new InstantCommand( ()->passthrough.shoot() ),
      new InstantCommand( ()->{} ), 
      ()->{return shooter.isOnTarget();}
    ));



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

    //Super basic get off the line
    // autos.buildSpinupAndShoot()
    // .andThen(()->shooter.setRPM(0))
    // .andThen(new ChassisDriveToHeadingBasic(1.5, ()->0, 3,0.05, navX, chassis))
    // ;

 
    Command fireCenteredAndDriveForward = 
    new ChassisDriveToHeadingBasic(0, ()->0, 3, 0.05, navX, chassis)
    .andThen(autos.buildSpinupAndShoot(110))//Estimated Guess of distance
    .andThen(()->shooter.setRPM(0))
    .andThen(new ChassisDriveToHeadingBasic(1.2, ()->0, 3, 0.05, navX, chassis))
    ;
    
    
    Command fullPortAuto = autos.buildSpinupAndShoot(103)
    .andThen(()->shooter.setRPM(0))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(-2.35, ()->0, 3, 0.05, navX, chassis))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(1.5, ()->0, 3, 0.05, navX, chassis))
    // .andThen(autos.buildSpinupAndShoot())
    // .andThen(()->shooter.setRPM(0))
    ;




    Command fullRondeAuto = new ChassisDriveToHeadingBasic(0, ()->20, 3, 0.05, navX, chassis)
    .andThen(autos.buildSpinupAndShoot(120))
    .andThen(()->shooter.setRPM(0))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-navX.getAngle(), 3, 0.05, navX, chassis))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(-1.65, ()->0, 3, 0.05, navX, chassis))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->37, 3, 0.05, navX, chassis))
    .andThen(new ChassisDriveToHeadingBasic(-1.2, ()->0, 3, 0.05, navX, chassis))
    .andThen(new IntakeDisengage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-21, 3, 0.05, navX, chassis))
    // .andThen(autos.buildSpinupAndShoot())
    // .andThen(()->shooter.setRPM(0))
    ;

    //Far
    Command fullFarTrenchRunAuto = new ChassisDriveToHeadingBasic(0, ()->70, 3, 0.05, navX, chassis)
    .andThen(autos.buildSpinupAndShoot(145))//Estimated Guess of distance
    .andThen(()->shooter.setRPM(0))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-navX.getAngle(), 3, 0.05, navX, chassis))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(-4.17, ()->0, 3, 0.05, navX, chassis)) //.alongWith(new IntakeEngage(intake))
    .andThen(new IntakeDisengage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(3, ()->0, 3, 0.05, navX, chassis)) //.alongWith(()->intake.disengage())
    .andThen(new ChassisDriveToHeadingBasic(0, ()->56, 3, 0.05, navX, chassis))
    // .andThen(autos.buildSpinupAndShoot())
    // .andThen(()->shooter.setRPM(0))
    ;


    //Near
    Command fullTrenchRunAuto = new ChassisDriveToHeadingBasic(0, ()->-32, 3, 0.05, navX, chassis)
    .andThen(autos.buildSpinupAndShoot(130))
    .andThen(()->shooter.setRPM(0))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-navX.getAngle(), 3, 0.05, navX, chassis))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(-4.17, ()->0, 3, 0.05, navX, chassis)) //.alongWith(new IntakeEngage(intake))
    .andThen(new IntakeDisengage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(3, ()->0, 3, 0.05, navX, chassis)) //.alongWith(()->intake.disengage())
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-27, 3, 0.05, navX, chassis))
    // .andThen(autos.buildSpinupAndShoot())
    // .andThen(()->shooter.setRPM(0))
    ;


     return fullFarTrenchRunAuto;
    
  }


  public Autos autos = new Autos();
  public class Autos{

    //This one's special: Because we may shoot multiple times, it's easier to just build it here and return it. 
    // All other movements are mostly one off or simple, so we don't need to do much.
    public ParallelDeadlineGroup buildSpinupAndShoot(double targetDistance){
      return    
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.02),
          new RunCommand(()->{}).withInterrupt(()->shooter.isOnTarget()).withTimeout(3),
          new InstantCommand(()->passthrough.shoot()),
          new WaitCommand(0.1),
          new RunCommand(()->{}).withInterrupt(()->passthrough.isOnTarget(1))
        ),
        new ShooterSetRPM(()->Constants.distanceToRPM.getOutputAt(targetDistance),shooter)
        //Distances for RPM 
        //130 near tennch
        //103 port
        //120 ronde
      );

    }

  }


}
