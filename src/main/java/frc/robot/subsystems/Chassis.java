/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  private CANSparkMax left = new CANSparkMax(1,MotorType.kBrushless);
  // private CANSparkMax leftA = new CANSparkMax(2,MotorType.kBrushless);
  // private CANSparkMax leftB = new CANSparkMax(3,MotorType.kBrushless);
  private CANSparkMax right = new CANSparkMax(4,MotorType.kBrushless);
  // private CANSparkMax rightA = new CANSparkMax(5,MotorType.kBrushless);
  // private CANSparkMax rightB = new CANSparkMax(6,MotorType.kBrushless);

  public DifferentialDrive drive;
  /**
   * Creates a new Chassis.
   */
  public Chassis() {
    //Invert master motors to ensure Positive motor values move forward, CW
    switch(Constants.botName){
      case TABI:
        left.setInverted(true);//TABI correction
        right.setInverted(true);//TABI correction
        //NOTE: We also want to invert the right encoder, but it's illegal, 
        // so we'll have to sort that out manually I guess.
      break;
      default:
    }
    drive = new DifferentialDrive(left, right);

    //Set up the follower motors
    // leftA.follow(left);
    // leftB.follow(left);
    // rightA.follow(right);
    // rightB.follow(right);

    //Configure the stall currents and limits
    int stallCurrent = 150/6;
    int freeCurrent = 240/6;
    int stallRPM = 5700/8;
    left.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);
    // leftA.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);
    // leftB.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);
    right.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);
    // rightA.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);
    // rightB.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);

    //Reset the encoders
    left.getEncoder().setPosition(0);
    right.getEncoder().setPosition(0);
    //Minimize motor timer overruns
    left.set(0);
    right.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
