/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.BotName;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;

public class ChassisVisionTargeting extends CommandBase {
  /**
   * Creates a new ChassisVisionTargeting.
   */

  Chassis chassis;
  Vision vision; 
  private AHRS gyro;

  MiniPID pidTurn = new MiniPID(0.015,0,0);

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

    pidTurn.reset();
    pidTurn.setSetpointRange(15); //TODO Find proper value //was 30
    pidTurn.setP(0.012);
    pidTurn.setI(0.003);
    pidTurn.setD(0.0015);
    pidTurn.setMaxIOutput(0.15);
    pidTurn.setOutputLimits(0.35);

    vision.targetPipeline();
    vision.lightsOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Direct vision backup method
    // double outputTurn = pidTurn.getOutput(0, vision.getTargetHeading());
  
    //preferred gyro method
    double outputTurn = -pidTurn.getOutput(gyro.getAngle(), vision.getTargetHeading());

    //Add a static feed-forward which makes things much more robust
    if(Constants.botName!=BotName.TABI){
      outputTurn += outputTurn>0 ? 0.08 : -0.08;
    }

    chassis.drive.arcadeDrive(0, outputTurn,false);

    //TODO Chassis inversion thing! Arcade drive backwards! 
    //keep an eye out for when it's fixed or not working - may need a "-"
    SmartDashboard.putNumber("vision/AimingOutput", outputTurn);
   
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
