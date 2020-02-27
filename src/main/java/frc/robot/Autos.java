/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChassisDriveToHeadingBasic;
import frc.robot.commands.ChassisVisionTargetingFancy;
import frc.robot.commands.IntakeDisengage;
import frc.robot.commands.IntakeEngage;
import frc.robot.commands.ShooterSetRPM;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public class Autos {

    public enum AutoName {SAFETY, BASIC_PORT_CENTERED, BASIC_NEAR_TRENCH, BASIC_FAR_TRENCH, BASIC_RONDE_CENTERED, FULL_NEAR_TRENCH, 
                          FULL_CENTERED_RONDE};

    private AHRS gyro;
    private Shooter shooter;
    private Intake intake;
    private Vision vision;
    private Passthrough passthrough;
    private Chassis chassis;

    
    private final double nearTrenchFiringAngle = -35; // degrees // also incorrect
    private final double nearTrenchFiringDistance = 170; // inches

    private final double farTrenchFiringAngle = 60; // degrees
    private final double farTrenchFiringDistance = 210; // inches

    private final double portCenteredFiringAngle = 35; // degrees
    private final double portCenteredFiringDistance = 150; // inches

    private final double rondeCenteredFiringAngle = 35; // degrees
    private final double rondeCenteredFiringDistance = 150; // inches

    private final double rondeAfterPickupFiringAngle = 35; // degrees
    private final double rondeAfterPickupFiringDistance = 210; // inches


    public Autos(AHRS gyro, Shooter shooter, Intake intake, Vision vision, Passthrough passthrough, Chassis chassis) {
        this.gyro = gyro;
        this.shooter = shooter;
        this.intake = intake;
        this.vision = vision;
        this.passthrough = passthrough;
        this.chassis = chassis;
    }




    // public Command getAuto(AutoName auto) {
    //     // switch(auto) {
    //     //     case SAFETY:

    //     //     return getSafety();

    //     //     break;

    //     //     case BASIC:

    //     //     return getBasic();

    //     //     break;

    //     //     case TRENCH:

    //     //     return getTrench();

    //     //     break;

    //     //     default:

    //     //     return getSafety();

    //     }

    public Command getSafety() {
        return new ChassisDriveToHeadingBasic(-1.5, () -> 0, 3, 0.05, gyro, chassis);
    }


    public Command turnAndShoot(double targetAngleToPort, double distanceToPort) {

        //alternative vision turning method

        // Command turnToTargetWithVision = new ChassisVisionTargetingFancy(vision, gyro, chassis).withTimeout(3);

        Command turnToTarget = new ChassisDriveToHeadingBasic(0, () -> targetAngleToPort, 3, 0.05, gyro, chassis);

        Command spinUpShooter1 = new ShooterSetRPM(() -> vision.getDistanceToRPMEmpirical(distanceToPort), shooter).withTimeout(2);


        Command aimAndSpinUp = new ParallelCommandGroup(
            turnToTarget, // turnToTargetWithVision, //
            spinUpShooter1
        );


        Command shoot = new RunCommand(()->passthrough.shoot(),passthrough).withInterrupt(()->passthrough.isOnTarget(4)).withTimeout(4);

        Command spinUpShooter2 = new ShooterSetRPM(() -> vision.getDistanceToRPMEmpirical(distanceToPort), shooter).withTimeout(2);


        Command fireAtSpeed = new ParallelCommandGroup(
            shoot,
            spinUpShooter2
        );


        Command rotatingToThenFire = new SequentialCommandGroup(
            aimAndSpinUp,
            fireAtSpeed
        );

        return rotatingToThenFire;
    };

        public Command turnAndShootHardSet(double targetAngleToPort, double distanceToPort) {

            Command turnToTarget = new ChassisDriveToHeadingBasic(0, () -> targetAngleToPort, 3, 0.05, gyro, chassis);

            Command spinUpShooter = new ShooterSetRPM(() -> vision.getDistanceToRPMEmpirical(distanceToPort), shooter).withTimeout(2);

            Command shoot = new RunCommand(()->passthrough.shoot(),passthrough).withInterrupt(()->passthrough.isOnTarget(4)).withTimeout(4);


            Command turnThenShoot = new SequentialCommandGroup(
                turnToTarget,
                shoot  
            );

            Command parallelSpinUpAndTurnThenShoot = new ParallelDeadlineGroup(
                turnThenShoot,
                spinUpShooter
            );

            return parallelSpinUpAndTurnThenShoot;

        }



    public Command turnAndShootAndResetAngle(double targetAngleToPort, double distanceToPort) {

        Command recoverAngle = new ChassisDriveToHeadingBasic(0, () -> -gyro.getAngle(), 3, 0.05, gyro, chassis);


        Command turnAndShootAndRecover = new SequentialCommandGroup(
            turnAndShoot(targetAngleToPort, distanceToPort),
            recoverAngle
        );
        
        return turnAndShootAndRecover;

    }

    public Command turnAndShootAndResetAngleAndDriveBack(double targetAngleToPort, double distanceToPort) {
        Command driveBack = new ChassisDriveToHeadingBasic(-1.5, () -> 0, 3, 0.05, gyro, chassis);

        Command rotateAndThenFireAndThenResetAngleAndThenFinallyDriveBackwards = new SequentialCommandGroup(
            turnAndShootAndResetAngle(targetAngleToPort, distanceToPort),
            driveBack
        );

        return rotateAndThenFireAndThenResetAngleAndThenFinallyDriveBackwards;
    }

    public Command getBasicNearTrench() {
        return turnAndShootAndResetAngleAndDriveBack(nearTrenchFiringAngle, nearTrenchFiringDistance);
    }

    public Command getBasicFarTrench() {
        return turnAndShootAndResetAngleAndDriveBack(farTrenchFiringAngle, farTrenchFiringDistance);
    }

    public Command getBasicPortCentered() {
        return turnAndShootAndResetAngleAndDriveBack(portCenteredFiringAngle, portCenteredFiringDistance);
    }

    public Command getBasicRondeCentered() {
        return turnAndShootAndResetAngleAndDriveBack(rondeCenteredFiringAngle, rondeCenteredFiringDistance);
    }



    public Command shootBalls(double distanceToPort) {


        Command spinUpShooter = new ShooterSetRPM(() -> vision.getDistanceToRPMEmpirical(distanceToPort), shooter).withTimeout(2);


        Command spinUp = new ParallelCommandGroup(
            spinUpShooter
        );


        Command shoot = new RunCommand(()->passthrough.shoot(),passthrough).withInterrupt(()->passthrough.isOnTarget(4)).withTimeout(4);


        Command fireAtSpeed = new ParallelCommandGroup(
            shoot,
            spinUpShooter
        );


        Command firing = new SequentialCommandGroup(
            spinUp,
            fireAtSpeed
        );

        return firing;
    };


    public Command trench3BallAuto() {
        
        Command intakeAndDriveBack = new ParallelDeadlineGroup(
            new ChassisDriveToHeadingBasic(-4.17, () -> 0, 3, 0.05, gyro, chassis),
            new IntakeEngage(intake)
        );

        Command disengageAndDriveBack = new ParallelDeadlineGroup(
            new ChassisDriveToHeadingBasic(3, () -> 0, 3, 0.05, gyro, chassis),
            new IntakeDisengage(intake)
        );


        Command trenchAuto = new SequentialCommandGroup(
            turnAndShootAndResetAngle(nearTrenchFiringAngle, nearTrenchFiringDistance),
            intakeAndDriveBack,
            disengageAndDriveBack
        );

        return trenchAuto;
        
    }  

    
    public Command ronde3BallAuto() {

        Command intakeAndDriveBack = new ParallelDeadlineGroup(
            new ChassisDriveToHeadingBasic(-2, () -> 0, 3, 0.05, gyro, chassis), //also needs good distance
            new IntakeEngage(intake)
        );


        Command turnAndDriveBackAndTurn = new SequentialCommandGroup( //TODO: Find/make values for angle/distances Rendezvous
            new ChassisDriveToHeadingBasic(-1.5, () -> 45, 3, 0.05, gyro, chassis), //currently guesses on angles/distances
            new ChassisDriveToHeadingBasic(0, () -> -45, 3, 0.05, gyro, chassis)
        );

        
        Command ronde3Ball = new SequentialCommandGroup(
            turnAndShootHardSet(rondeCenteredFiringAngle, rondeCenteredFiringDistance),
            intakeAndDriveBack,
            turnAndDriveBackAndTurn,
            new IntakeDisengage(intake),
            turnAndShootAndResetAngleAndDriveBack(rondeAfterPickupFiringAngle, rondeAfterPickupFiringDistance)
        );

        return ronde3Ball;

    }




    // public Command getBasic() {

        // Command aimAndGetToSpeed = new ParallelCommandGroup(
        //     new ChassisVisionTargeting(vision, gyro, chassis)
        //         .withTimeout(4)
        //         .withInterrupt( ()->{ return Math.abs(vision.getTargetHeading())<4; } ),

        //     new ShooterSetRPM(() -> 2000, shooter) // the rpm will be set later
        // );

        

        // Command basicAuto = new SequentialCommandGroup(
        //     aimAndGetToSpeed




        // );

        // return basicAuto;
    // }







    
    
    // Command aimAndGetToSpeed = new ParallelCommandGroup(
    //   new ShooterSetRPM(()->2000, shooter).withTimeout(4),

    // new ChassisDriveToHeadingBasic(0, () -> -35, 3, 0.05, navX, chassis)

    //   // new ChassisVisionTargeting(vision, navX, chassis)
    //   //   .withTimeout(4)
    //   //   .withInterrupt( ()->{ return Math.abs(vision.getTargetHeading())<4; } )
    // );

    // Command fireAtSpeed = new ParallelDeadlineGroup
    // (
    //   new RunCommand(()->passthrough.shoot(),passthrough).withInterrupt(()->passthrough.isOnTarget(4)).withTimeout(6),
    //   new ShooterSetRPM(() -> 2000, shooter).withTimeout(8)
    // );

    // Command resetPositionAndShooter = new ParallelCommandGroup(
    //   // new ShooterSetRPM(()->0, shooter).withTimeout(0),
    //   new ChassisDriveToHeadingBasic(0, () -> -navX.getAngle(), 3 /*Degrees*/, 0.05 /*Meters*/, navX, chassis) // the turn back to straight
    //   );

    //   Command intakeAndDriveBack = new ParallelCommandGroup(
    //     new IntakeEngage(intake),
    //     new ChassisDriveToHeadingBasic(-4.17, () -> 0, 3 /*Degrees*/, 0.05 /*Meters*/, navX, chassis)
    //   );
  
    //   Command stopIntakeAndDriveForward = new ParallelCommandGroup(
    //     new IntakeDisengage(intake),
    //     new ChassisDriveToHeadingBasic(4, () -> 0, 3, 0.05, navX, chassis)
    //   );
  
    //   // Command disengageIntake = new IntakeDisengage(intake);
  
      
    //   Command  autoFromTrenchAlignment = new SequentialCommandGroup(
  
    //     aimAndGetToSpeed, // then
  
    //     fireAtSpeed, // and then
  
    //     resetPositionAndShooter, // and then
  
    //     intakeAndDriveBack, // and then
        
    //     stopIntakeAndDriveForward
    //   );
  
    //   autoCommand = autoFromTrenchAlignment;
      


    
}
