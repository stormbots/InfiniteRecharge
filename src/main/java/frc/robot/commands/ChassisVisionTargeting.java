/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;

public class ChassisVisionTargeting extends CommandBase {
  /**
   * Creates a new ChassisVisionTargeting.
   */

  Chassis chassis;
  Vision vision; 
  private AHRS gyro;

  public ChassisVisionTargeting(Vision vision, AHRS navX, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    this.vision = vision;
    this.gyro = navX;
    this.chassis = chassis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //TODO: We shouldn't need to reset for vision to work right and apparently it'll mess up Zach and his autos
    //gyro.reset();

    //pidTurn = chassis.getPID();

    vision.pidTurn.reset();
    
    vision.targetPipeline();
    vision.lightsOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if( vision.isTargetValid()==false){
      chassis.drive.arcadeDrive(0, 0, false);
      return;
    }
    
    //Direct vision backup method
    // double outputTurn = pidTurn.getOutput(0, vision.getTargetOffset());
  
    //preferred gyro method
    double outputTurn = vision.pidTurn.getOutput(gyro.getAngle(), vision.getTargetHeading());

    //Add a static feed-forward which makes things much more robust
    chassis.drive.arcadeDrive(0, outputTurn,false);

    //TODO Chassis inversion thing! Arcade drive backwards! 
    //keep an eye out for when it's fixed or not working - may need a "-"
    SmartDashboard.putNumber("vision/AimingOutput", outputTurn);
    SmartDashboard.putNumber("vision/Gyro Angle", gyro.getAngle());
   
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
