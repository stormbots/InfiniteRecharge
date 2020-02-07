/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import com.kauailabs.navx.frc.AHRS;
import com.stormbots.closedloop.FB;
import com.stormbots.interp.SinCurve;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDriveToHeadingBasic extends CommandBase {
  private final Chassis chassis;
  private AHRS gyro;
  private double forwardDistance;
  private double targetBearing;
  private double initialBearing;


  
  SlewRateLimiter speedslew;


  /**
   * Creates a new ChassisDriveManual.
   * All distance units should be in Meters
   */
  public ChassisDriveToHeadingBasic(double targetDistance, double targetBearing, AHRS gyro, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.gyro = gyro;
    this.forwardDistance = targetDistance;
    this.targetBearing = targetBearing;
    addRequirements(chassis);

    speedslew = new SlewRateLimiter(chassis.ACCEL_DISTANCE, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.getLeftEncoder().setPosition(0);
    chassis.getRightEncoder().setPosition(0);

    gyro.reset();
    
    initialBearing = gyro.getAngle();
    chassis.getPID().setSetpoint(initialBearing + targetBearing);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentAngle =  gyro.getAngle();
    //Uses PID to create a motion controlled turn value
    double turn = chassis.getPID().getOutput(currentAngle);
    
    double distance = chassis.getAverageDistance();

    double targetDistance = speedslew.calculate(forwardDistance);

    double forwardSpeed = FB.fb(targetDistance, distance, 0.4);

    if(Math.abs(targetBearing - currentAngle) > 20) {
      forwardSpeed = 0;
    }

    // forwardSpeed = 0; //DEBUG

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
