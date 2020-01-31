/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;

public class ChassisVisionTargeting extends CommandBase {
  /**
   * Creates a new ChassisVisionTargeting.
   */

  Chassis chassis;
  Vision vision; 
  private Gyro gyro;

  MiniPID pidTurn = new MiniPID(0.015,0,0);

  public ChassisVisionTargeting(Vision vision, Gyro gyro, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.vision = vision;
    this.gyro = gyro;
    this.chassis = chassis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pidTurn.setSetpointRange(30); //TODO Find proper value
    pidTurn.setOutputLimits(0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //directly off limelight
    //double output = pid.getOutput(x, 0);
    //driver.arcadeDrive(0, output);

    vision.targetPipeline();
    vision.lightsOn();
    
    double outputTurn = pidTurn.getOutput(gyro.getAngle(), vision.getTargetHeading());
    chassis.drive.arcadeDrive(0, outputTurn); 
    //TODO Chassis inversion thing! Arcade drive backwards! 
    //keep an eye out for when it's fixed or not working - may need a "-"
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.driverPipeline();
    vision.lightsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
