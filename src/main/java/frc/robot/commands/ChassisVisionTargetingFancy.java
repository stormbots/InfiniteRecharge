/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.BotName;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Vision;

public class ChassisVisionTargetingFancy extends CommandBase {
  /**
   * Creates a new ChassisVisionTargeting.
   */

  Chassis chassis;
  Vision vision; 
  private AHRS gyro;

  MiniPID pidTurn = new MiniPID(0.015,0,0);

  Double targetHeading = 0.0;
  private boolean foundValidTarget;

  public ChassisVisionTargetingFancy(Vision vision, AHRS navX, Chassis chassis) {
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

    pidTurn = chassis.getPID();

    pidTurn.reset();
    pidTurn.setSetpointRange(15); //TODO Find proper value //was 30
    pidTurn.setP(0.013);
    pidTurn.setI(0.001);
    // pidTurn.setD(0.0015);
    pidTurn.setMaxIOutput(0.15);
    pidTurn.setOutputLimits(0.35);
    pidTurn.setF((s,a,e)->{return Math.abs(e)*0.035;/*static FeedForward*/ });

    vision.targetPipelineFancy();
    vision.lightsOn();

    foundValidTarget=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Direct vision backup method
    // double outputTurn = pidTurn.getOutput(0, vision.getTargetHeading());

    if(!foundValidTarget){
      //look for target
      if(vision.isTargetValid() && NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0)>0){
        foundValidTarget = true;
        targetHeading = gyro.getAngle() + NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      }
      return;
    }

  
    //preferred gyro method
    double outputTurn = pidTurn.getOutput(gyro.getAngle(), targetHeading);
    // SmartDashboard.putNumber("vision/FancyTargetAngle",targetHeading);
    // SmartDashboard.putNumber("vision/FancyGyro",gyro.getAngle());
    // SmartDashboard.putNumber("vision/FancyOutput",outputTurn);

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
