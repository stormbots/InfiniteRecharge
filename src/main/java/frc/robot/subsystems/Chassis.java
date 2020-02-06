/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BotName;

public class Chassis extends SubsystemBase {

  // private AHRS navX = new AHRS(SPI.Port.kMXP);

  private CANSparkMax left = new CANSparkMax(1,MotorType.kBrushless);
  // private CANSparkMax leftA = new CANSparkMax(2,MotorType.kBrushless);
  // private CANSparkMax leftB = new CANSparkMax(3,MotorType.kBrushless);
  private CANSparkMax right = new CANSparkMax(4,MotorType.kBrushless);
  // private CANSparkMax rightA = new CANSparkMax(5,MotorType.kBrushless);
  // private CANSparkMax rightB = new CANSparkMax(6,MotorType.kBrushless);

  public DifferentialDrive drive;

  // public Solenoid shifter = new Solenoid(2);
  // public Solenoid shifterInverse = new Solenoid(5);



  // Use an Enum to define pnuematic truth values, so that you get good named values 
  // backed by type checking everywhere.
  public enum Gear{
    HIGH(true,true),
    LOW(false,false);
    private boolean compbot,practicebot;
    Gear(boolean compbot, boolean practicebot){
      this.compbot = compbot;
      this.practicebot = practicebot;
    }
    public boolean bool(){return Constants.botName == BotName.COMP ? this.compbot : this.practicebot;};
  }



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
        //convert to Meters and Meters/second
        left.getEncoder().setPositionConversionFactor(Math.PI*0.105*14/70);
        right.getEncoder().setPositionConversionFactor(Math.PI*0.105*14/70);
        // left.getEncoder().setVelocityConversionFactor(Math.PI*0.105*14/70/60);
        // right.getEncoder().setVelocityConversionFactor(Math.PI*0.105*14/70/60);
      break;

      case PRACTICE:
        left.setInverted(true);
        right.setInverted(true);
        left.getEncoder().setPositionConversionFactor(Math.PI*6*Constants.METERS_TO_INCHES*(1/18.75));
        right.getEncoder().setPositionConversionFactor(Math.PI*6*Constants.METERS_TO_INCHES*(1/18.75));
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

  
  public void shift(Gear gear){
    // shifter.set(gear.bool());
    // shifterInverse.set(!gear.bool());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getAverageDistance());
    
  }

  public CANEncoder getLeftEncoder() {
    return left.getEncoder();
  }

  public CANEncoder getRightEncoder() {
    return right.getEncoder();
  }

  public double getAverageDistance() {
    return (left.getEncoder().getPosition() - right.getEncoder().getPosition())/2.0;
  }

}
