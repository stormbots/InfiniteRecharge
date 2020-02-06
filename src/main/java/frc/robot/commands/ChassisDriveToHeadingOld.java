/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.closedloop.MiniPID;
import com.stormbots.interp.SinCurve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import static frc.robot.Constants.*;

public class ChassisDriveToHeadingOld extends CommandBase {
  private final Chassis chassis;
  private AHRS gyro;
  private double forwardDistance;
  private double targetHeading;
  private double initialPosition;
  private double bearing;
  private double accelDistance = 6*INCHES; //Dan likes inches -> conversion in constants
  // private Lerp angleToPower = new Lerp(-180, 180, -1, 1);
  private MiniPID pid = new MiniPID(0, 0, 0); //TODO: Needs to be tuned

  /**
   * Creates a new ChassisDriveManual.
   * All distance units should be in Meters
   */
  public ChassisDriveToHeadingOld(double targetDistance, double targetHeading, AHRS gyro, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.gyro = gyro;
    this.forwardDistance = targetDistance;
    this.targetHeading = targetHeading;
    addRequirements(chassis);
    pid.setSetpoint(targetHeading);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = chassis.getLeftEncoder().getPosition();
    forwardDistance += initialPosition;
    bearing = gyro.getAngle() - 180;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentAngle =  gyro.getAngle();
    //Uses PID to create a motion controlled turn value
    double turn = pid.getOutput(currentAngle-bearing);
    
    double distance = Math.hypot(gyro.getDisplacementX(), gyro.getDisplacementY());

    //If need to print out gyro
    //System.out.println(gyro.getAngle());

    double forwardSpeed = SinCurve.scurve(distance, initialPosition, initialPosition+accelDistance, 0, 1);
    forwardSpeed -= SinCurve.scurve(distance, forwardDistance - accelDistance, forwardDistance, 0,1);

    chassis.drive.arcadeDrive(
      forwardSpeed, 
      turn,
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    chassis.drive.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//(encoder.getAsDouble() >= initialPosition+forwardDistance);
  }
}
