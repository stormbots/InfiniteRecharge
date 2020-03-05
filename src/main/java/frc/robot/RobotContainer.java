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
  public final Vision vision = new Vision(navX);
  

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
  Button climbHookRetract = new JoystickButton(controller, 9);
  Button climbHookGrab = new JoystickButton(controller,8);
  Button lowGoalShoot = new JoystickButton(controller, 10);

  //Debug 
  // Button climbTranslateRight = new JoystickButton(controller, 10);
  // Button climbTranslateSideways = new JoystickButton(controller, 11);

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

    buildAutoCommands();

    configAutoSelector();
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
    // visionAimToTarget.whileHeld(new ChassisVisionTargeting(vision, navX, chassis));
    visionAimToTarget.whileHeld( new ChassisVisionTargeting(
      ()->-driver.getRawAxis(1)*Math.abs(driver.getRawAxis(1))*0.5 ,
      vision, navX, chassis));
    visionAimToTargetFancy.whileHeld(new ChassisVisionTargetingFancy(vision, navX, chassis));
    ;
    //Zach wanted to have 75% output on forward
    fineTurning.whileHeld(new ChassisDriveManual(() -> 0.75 * driver.getRawAxis(1), () -> 0.5 * driver.getRawAxis(2), chassis));

    /* Player 2 normal Buttons */
    intakeButton.whenPressed(new IntakeEngage(intake));
    intakeButton.whenReleased(new IntakeDisengage(intake).withTimeout(0.1));

    shooterSpinDefaultSpeed.whenPressed(()->passthrough.prepareForShooting());
    shooterSpinDefaultSpeed.whenPressed( 
      new ShooterSetRPM( ()->Constants.distanceToRPM.getOutputAt(20*12), false, shooter) 
    );
    shooterSpinDefaultSpeed.whenReleased( 
      new ShooterSetRPM( ()->Constants.distanceToRPM.getOutputAt(20*12), shooter).withTimeout(1.5)
    );
    // shooterSpinDefaultSpeed.whileHeld(new ShooterSetRPM(()->SmartDashboard.getNumber("shooter/RMPDebugSet", 4000), shooter));
    shooterSpinDefaultSpeed.whenReleased(()->passthrough.reset());

    shooterSpinCalculatedSpeed.whenPressed(()->passthrough.prepareForShooting());
    shooterSpinCalculatedSpeed.whileHeld(new ShooterSetRPM(()->{
      if( vision.isTargetValid() ){ return Constants.distanceToRPM.getOutputAt(vision.getDistance()); } 
      else { return Constants.distanceToRPM.getOutputAt(20*12);}
    }, shooter));
    shooterSpinCalculatedSpeed.whenReleased( 
      new ShooterSetRPM(()->{
        if( vision.isTargetValid() ){ return Constants.distanceToRPM.getOutputAt(vision.getDistance()); } 
        else { return Constants.distanceToRPM.getOutputAt(20*12);}
      }, shooter).withTimeout(1.5)
    );
    // shooterSpinCalculatedSpeed.whileHeld( new ShooterSetRPM( ()->7500, shooter) );
    shooterSpinCalculatedSpeed.whenReleased(()->passthrough.reset());

    
    // lowGoalShoot.whenPressed(()->passthrough.prepareForShooting());
    // lowGoalShoot.whileHeld(new ShooterSetRPM(()->3000, shooter));
    // lowGoalShoot.whenReleased(()->passthrough.reset());

    // conditional
    // shoot.whileHeld(new RunCommand( ()->{
    //   if(shooter.isOnTarget() && shooter.getRPM() > 500)passthrough.shoot();
    // }));
    //   new InstantCommand( ()->passthrough.shoot() ),
    //   new InstantCommand( ()->{} ), 
    //   ()->{return shooter.isOnTarget() && shooter.getRPM() > 500;}
    // ));
    shoot.whileHeld(()-> passthrough.shoot());



    loadBallManually.whenPressed(new InstantCommand(()->passthrough.loadBall(),passthrough));
    eject.whenPressed(new PassthroughEject(passthrough, intake));

    /*Player 2 Climb Buttons */

    climbHookRetract.whenPressed(()->{
      if(!climbEnable.get())return;
      climber.setHookAngle(100);
    });
    climbHookGrab.whenPressed(()->{
      if(!climbEnable.get())return;
      climber.setHookAngle(240);//Hook angle thinks that this is 175 degrees i think, but it is about 180 (- some for safety)
      //Looks to be magnitutude of 10 off
    });
    //TODO: Not implemented in hardware yet
    // translationMoveForwards.whenPressed(new ClimberSetTranslation(()->0.2, climber));
    // translationMoveBackwards.whenPressed(new ClimberSetTranslation(()->-0.2,climber));

    /*Debug Buttons */ //TODO: Remove these before competitions
    // tempClimbSpoolPositive.whileHeld(new SpinSpoolPositive(climber));
    // tempClimbSpoolNegative.whileHeld(new SpinSpoolNegative(climber));


    climbEnable.whileHeld(
      new ClimbManual(()->climbEnable.get(),()->controller.getRawAxis(3),climber)
    );

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
    // climber.setDefaultCommand(
    //   new ClimbManual(()->climbEnable.get(),()->controller.getRawAxis(3),climber)
    // );

    //Ensure that after vision is used, it returns to Driver view
    vision.setDefaultCommand( new FunctionalCommand(
      ()->vision.driverPipeline(),
      ()->{}, 
      (interrupted)->{}, 
      ()->false,
      vision
    ));

    // shooter.setDefaultCommand(new ShooterSetRPM( ()->0, shooter));
    
  }


  SendableChooser<Command> autoChooser = new SendableChooser<>();
  SendableChooser<Boolean> limelightInAuto = new SendableChooser<>();

  public void configAutoSelector(){
    autoChooser.setDefaultOption("Basic", fireCenteredAndDriveForward);

    autoChooser.addOption("FarTrench", fullFarTrenchRunAuto);
    autoChooser.addOption("VisionFarTrench", fullFarTrenchRunAutoVision);

    autoChooser.addOption("NearTrench", fullNearTrenchRunAuto);
    autoChooser.addOption("VisionNearTrench", fullNearTrenchRunAutoVision);

    autoChooser.addOption("Rondezvous", fullRondeAuto);
    autoChooser.addOption("VisionRondezvous", fullRondeAutoVision);

    autoChooser.addOption("Port", fullPortAuto);
    autoChooser.addOption("VisionPort", fullPortAutoVision);

    SmartDashboard.putData("autos/position", autoChooser);
    

    limelightInAuto.setDefaultOption("No", false);
    limelightInAuto.addOption("Yes", true);
    SmartDashboard.putData("autos/limelight", limelightInAuto);    

  }

  //-25.5 trenchnear


  Command fireCenteredAndDriveForward;

  Command fullFarTrenchRunAuto;
  Command fullFarTrenchRunAutoVision;

  Command fullNearTrenchRunAuto;
  Command fullNearTrenchRunAutoVision;

  Command fullRondeAuto;
  Command fullRondeAutoVision;

  Command fullPortAuto;
  Command fullPortAutoVision;



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //get values from dashboard
    return autoChooser.getSelected();
  }

  public void buildAutoCommands() {

    //Super basic get off the line
    // autos.buildSpinupAndShoot()
    // .andThen(()->shooter.setRPM(0))
    // .andThen(new ChassisDriveToHeadingBasic(1.5, ()->0, 3,0.05, navX, chassis))
    // ;

    // Basic
    fireCenteredAndDriveForward = 
    new ChassisDriveToHeadingBasic(0, ()->0, 3, 0.05, navX, chassis)
    .andThen(autos.buildSpinupAndShoot(110))//Estimated Guess of distance
    .andThen(()->shooter.setRPM(0))
    .andThen(new ChassisDriveToHeadingBasic(1.2, ()->0, 3, 0.05, navX, chassis))
    ;

    
    // Port
    fullPortAuto = autos.buildSpinupAndShoot(120) // 103
    .andThen(()->shooter.setRPM(0))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(-2.35, ()->0, 3, 0.05, navX, chassis))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(1.5, ()->0, 3, 0.05, navX, chassis))
    // .andThen(autos.buildSpinupAndShoot())
    // .andThen(()->shooter.setRPM(0))
    ;
    
    // Port with Limelight Vision
    fullPortAutoVision = new ChassisDriveToHeadingBasic(0, ()->-6, 2, 0.05, navX, chassis)
    .andThen(autos.buildSpinupAndShoot(104))
    .andThen(()->shooter.setRPM(0))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-navX.getAngle(), 3, 0.05, navX, chassis))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new InstantCommand(()->vision.targetPipelineFancy(), vision) )
    .andThen(new InstantCommand(()->chassis.ACCEL_DISTANCE = 7))
    .andThen(new ChassisDriveToHeadingBasic(-2.35+0.5, ()->0, 3, 0.05, navX, chassis))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->25, 3, 0.05, navX, chassis))
    .andThen(new InstantCommand(()->chassis.ACCEL_DISTANCE = 1))
    .andThen(new ChassisDriveToHeadingBasic(-1, ()->0, 3, 0.05, navX, chassis))
    .andThen(new ChassisDriveToHeadingBasic(1, ()->0, 3, 0.05, navX, chassis))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-navX.getAngle(), 3, 0.05, navX, chassis))
    .andThen(new IntakeDisengage(intake).withTimeout(0.02))
    // .andThen(new ChassisDriveToHeadingBasic(1.5, ()->0, 3, 0.05, navX, chassis))
    .andThen(new ChassisVisionTargetingFancy(vision, navX, chassis).withTimeout(2))
    .andThen(autos.buildSpinupAndShoot(220))
    .andThen(()->shooter.setRPM(0))
    ;


    // Ronde
    fullRondeAuto = new ChassisDriveToHeadingBasic(0, ()->20, 3, 0.05, navX, chassis)
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

    // Ronde with Limelight vision
    fullRondeAutoVision = new ChassisDriveToHeadingBasic(0, ()->16, 3, 0.05, navX, chassis)
    .andThen(autos.buildSpinupAndShoot(120))
    .andThen(()->shooter.setRPM(0))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-navX.getAngle(), 3, 0.05, navX, chassis))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new InstantCommand(()->chassis.ACCEL_DISTANCE = 7))
    .andThen(new ChassisDriveToHeadingBasic(-1.75-.2, ()->0, 3, 0.05, navX, chassis))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->37, 3, 0.05, navX, chassis))
    .andThen(new InstantCommand(()->vision.targetPipelineFancy(), vision) )
    .andThen(new InstantCommand(()->chassis.ACCEL_DISTANCE = 1))
    .andThen(new ChassisDriveToHeadingBasic(-0.6, ()->0, 3, 0.05, navX, chassis))
    .andThen(new InstantCommand(()->chassis.ACCEL_DISTANCE = 7))
    .andThen(new IntakeDisengage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(0.55, ()->0, 3, 0.05, navX, chassis))
    // .andThen(new ChassisDriveToHeadingBasic(0, ()->-21, 3, 0.05, navX, chassis))
    .andThen(new ChassisVisionTargetingFancy(vision, navX, chassis).withTimeout(2))
    .andThen(autos.buildSpinupAndShoot(220))
    .andThen(()->shooter.setRPM(0))
    ;


    //Far
    fullFarTrenchRunAuto = new ChassisDriveToHeadingBasic(0, ()->70, 3, 0.05, navX, chassis)
    // .andThen(autos.buildSpinupAndShoot(130))//Estimated Guess of distance
    // .andThen(()->shooter.setRPM(0))
    // .andThen(new ChassisDriveToHeadingBasic(0, ()->-navX.getAngle(), 3, 0.05, navX, chassis))
    // .andThen(new IntakeEngage(intake).withTimeout(0.02))
    // .andThen(new ChassisDriveToHeadingBasic(-4.17, ()->0, 3, 0.05, navX, chassis)) //.alongWith(new IntakeEngage(intake))
    // .andThen(new IntakeDisengage(intake).withTimeout(0.02))
    // .andThen(new ChassisDriveToHeadingBasic(3, ()->0, 3, 0.05, navX, chassis)) //.alongWith(()->intake.disengage())
    // .andThen(new ChassisDriveToHeadingBasic(0, ()->56, 3, 0.05, navX, chassis))
    // // .andThen(autos.buildSpinupAndShoot())
    // // .andThen(()->shooter.setRPM(0))
    ;

    //Far with Limelight Vision
    fullFarTrenchRunAutoVision = new ChassisDriveToHeadingBasic(0, ()->70, 3, 0.05, navX, chassis)
    // .andThen(autos.buildSpinupAndShoot(130))//Estimated Guess of distance
    // .andThen(()->shooter.setRPM(0))
    // .andThen(new ChassisDriveToHeadingBasic(0, ()->-navX.getAngle(), 3, 0.05, navX, chassis))
    // .andThen(new IntakeEngage(intake).withTimeout(0.02))
    // .andThen(new ChassisDriveToHeadingBasic(-3.3, ()->0, 3, 0.05, navX, chassis)) //.alongWith(new IntakeEngage(intake))
    // .andThen(new IntakeDisengage(intake).withTimeout(0.02))
    // .andThen(new InstantCommand(()->vision.targetPipelineFancy(), vision) )
    // .andThen(new ChassisDriveToHeadingBasic(2.3, ()->0, 3, 0.05, navX, chassis)) //.alongWith(()->intake.disengage())
    // // .andThen(new ChassisDriveToHeadingBasic(0, ()->56, 3, 0.05, navX, chassis))
    // .andThen(new ChassisVisionTargetingFancy(vision, navX, chassis).withTimeout(2))
    // .andThen(autos.buildSpinupAndShoot(180))
    // .andThen(()->shooter.setRPM(0))
    ;


    //Near
    fullNearTrenchRunAuto = new ChassisDriveToHeadingBasic(0, ()->-25.5, 3, 0.05, navX, chassis)
    .andThen(autos.buildSpinupAndShoot(130))
    .andThen(()->shooter.setRPM(0))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-navX.getAngle(), 3, 0.05, navX, chassis))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new ChassisDriveToHeadingBasic(-4.67, ()->0, 3, 0.05, navX, chassis)) //.alongWith(new IntakeEngage(intake))
    .andThen(new IntakeDisengage(intake).withTimeout(0.02))
    .andThen(new InstantCommand(()->vision.targetPipelineFancy(), vision) )
    .andThen(new ChassisDriveToHeadingBasic(3.5, ()->0, 3, 0.05, navX, chassis)) //.alongWith(()->intake.disengage())
    // .andThen(new ChassisDriveToHeadingBasic(0, ()->-27, 3, 0.05, navX, chassis))
    .andThen(new ChassisVisionTargetingFancy(vision, navX, chassis).withTimeout(2))
    // .andThen(autos.buildSpinupAndShoot(220))
    // .andThen(()->shooter.setRPM(0))
    ;

    //Near with Limelight vision
    fullNearTrenchRunAutoVision = new ChassisDriveToHeadingBasic(0, ()->-25.5, 3, 0.05, navX, chassis)
    .andThen(autos.buildSpinupAndShoot(130))
    .andThen(()->shooter.setRPM(0))
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-navX.getAngle(), 3, 0.05, navX, chassis))
    .andThen(new IntakeEngage(intake).withTimeout(0.02))
    .andThen(new InstantCommand(()->chassis.ACCEL_DISTANCE = 6))
    .andThen(new ChassisDriveToHeadingBasic(-2.5, ()->0, 3, 0.05, navX, chassis)) //.alongWith(new IntakeEngage(intake))
    .andThen(new InstantCommand(()->chassis.ACCEL_DISTANCE = 1))
    .andThen(new ChassisDriveToHeadingBasic(-4.67+2.5, ()->0, 3, 0.05, navX, chassis)) //.alongWith(new IntakeEngage(intake))
    .andThen(new InstantCommand(()->chassis.ACCEL_DISTANCE = 12))
    .andThen(new IntakeDisengage(intake).withTimeout(0.02))
    // .andThen(new InstantCommand(()->vision.targetPipeline(), vision) )
    .andThen(new InstantCommand(()->shooter.setRPM(4000)))
    .andThen(new ChassisDriveToHeadingBasic(3.5, ()->0, 3, 0.05, navX, chassis)) //.alongWith(()->intake.disengage())
    .andThen(new ChassisDriveToHeadingBasic(0, ()->-24+3+2, 3, 0.05, navX, chassis))
    // .andThen(new ChassisVisionTargeting(vision, navX, chassis)
      // .withInterrupt(()->vision.isOnTarget(2))
      // .withTimeout(1)
      // .alongWith(new ShooterSetRPM(()->Constants.distanceToRPM.getOutputAt(180),false,shooter))
    // )
    .andThen(autos.buildSpinupAndShoot(180))
    .andThen(()->shooter.setRPM(0))
    ;


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
