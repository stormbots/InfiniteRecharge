/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BotName;

public class Chassis extends SubsystemBase {

  public final double kP;
  public final double kD;

  public final double kS;
  public final double kV;
  public final double kA;

  public final double maxChassisVelocity;


  private MiniPID turningPID;

  private CANSparkMax left = new CANSparkMax(1,MotorType.kBrushless);
  private CANSparkMax leftA = new CANSparkMax(2,MotorType.kBrushless);
  private CANSparkMax leftB = new CANSparkMax(3,MotorType.kBrushless);
  private CANSparkMax right = new CANSparkMax(4,MotorType.kBrushless);
  private CANSparkMax rightA = new CANSparkMax(5,MotorType.kBrushless);
  private CANSparkMax rightB = new CANSparkMax(6,MotorType.kBrushless);

  public DifferentialDrive drive;

  public double LEFT_INVERSION;
  public double RIGHT_INVERSION;
  public double ACCEL_DISTANCE;

  public Solenoid shifter = new Solenoid(1);
  public Solenoid shifterInverse = new Solenoid(5);



  // Use an Enum to define pnuematic truth values, so that you get good named values 
  // backed by type checking everywhere.
  public enum Gear{
    HIGH(true, true),
    LOW(false, false);
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
    switch(Constants.botName){
      case PRACTICE:
        //left and right motors are swapped
        left = new CANSparkMax(4,MotorType.kBrushless);
        leftA = new CANSparkMax(5,MotorType.kBrushless);
        leftB = new CANSparkMax(6,MotorType.kBrushless);
        right = new CANSparkMax(1,MotorType.kBrushless);
        rightA = new CANSparkMax(2,MotorType.kBrushless);
        rightB = new CANSparkMax(3,MotorType.kBrushless);

        LEFT_INVERSION = 1;
        RIGHT_INVERSION = -1;
      break;
      case COMP:
        //456 on right
        //123 on left
        left = new CANSparkMax(1,MotorType.kBrushless);
        leftA = new CANSparkMax(2,MotorType.kBrushless);
        leftB = new CANSparkMax(3,MotorType.kBrushless);
        right = new CANSparkMax(4,MotorType.kBrushless);
        rightA = new CANSparkMax(5,MotorType.kBrushless);
        rightB = new CANSparkMax(6,MotorType.kBrushless);
        LEFT_INVERSION = -1;
        RIGHT_INVERSION = 1;
      break;
      case TABI: 
        left = new CANSparkMax(1,MotorType.kBrushless);
        leftA = new CANSparkMax(2,MotorType.kBrushless);
        leftB = new CANSparkMax(3,MotorType.kBrushless);
        right = new CANSparkMax(4,MotorType.kBrushless);
        rightA = new CANSparkMax(5,MotorType.kBrushless);
        rightB = new CANSparkMax(6,MotorType.kBrushless);
        LEFT_INVERSION = -1;
        RIGHT_INVERSION = 1;
    }

    left.setIdleMode(IdleMode.kBrake);
    leftA.setIdleMode(IdleMode.kCoast);
    leftB.setIdleMode(IdleMode.kCoast);
    right.setIdleMode(IdleMode.kBrake);
    rightA.setIdleMode(IdleMode.kCoast);
    rightB.setIdleMode(IdleMode.kCoast);

    switch(Constants.botName){
      case COMP:
      //TODO: Calculate properly: Using practice bot as reference temporarily
      kP = 0.551;
      kD = 0.0;
    
      kS = 0.166;
      kV = 0.121;
      kA = 0.0122;
    
      maxChassisVelocity = 2;
      break;

      case PRACTICE: // THE PROPER GEARING FOUND EXPERIMENTALLY IS 4.76/10 or 0.476
      kP = 0.551;
      kD = 0.0;
    
      kS = 0.166;
      kV = 0.121;
      kA = 0.0122;
    
      maxChassisVelocity = 2;
      break;

      case TABI: //falls through to default
      default://default needed to make compiler happy
      kP = 0.0;//507;//13.5;//
      kD = 0.0;
    
      kS = 0.152;//0.128;
      kV = 1.98;//0.077;
      kA = 0;//0.354;//0.0109;
    
      maxChassisVelocity = 3;
      break;

    }


    //Invert master motors to ensure Positive motor values move forward, CW
    switch(Constants.botName){
      case TABI:
        left.setInverted(true);//TABI correction
        right.setInverted(true);//TABI correction
        left.getEncoder().setPositionConversionFactor(Math.PI*0.105*14/70);
        right.getEncoder().setPositionConversionFactor(Math.PI*0.105*14/70);
        left.getEncoder().setVelocityConversionFactor(Math.PI*0.105*14/70/60);
        right.getEncoder().setVelocityConversionFactor(Math.PI*0.105*14/70/60);

        turningPID = new MiniPID(0.2/30, 0, 0.0)
        .setI(0.05/200.0)
        .setOutputLimits(0.2)
        .setMaxIOutput(0.15)
        .setSetpointRange(30)
        // .setF((s,a,e)->{
        //   return Math.signum(e)*0.08;
        // }) //FIXME: this should work but doesnt
        ;

        ACCEL_DISTANCE = 0.25;
      break;

      case PRACTICE:
        left.setInverted(true);
        leftA.setInverted(true);
        leftB.setInverted(true);
        right.setInverted(true);
        rightA.setInverted(true);
        rightB.setInverted(true);

        left.getEncoder().setPositionConversionFactor(7.3914 / 287.4546);
        right.getEncoder().setPositionConversionFactor(7.3914 / 287.4546);
        left.getEncoder().setVelocityConversionFactor(7.3914 / 287.4546 / 60);
        right.getEncoder().setVelocityConversionFactor(7.3914 / 287.4546 / 60);

        // left.getEncoder().setPositionConversionFactor(Math.PI*0.152*(1/0.476));
        // right.getEncoder().setPositionConversionFactor(Math.PI*0.152*(1/0.476));
        // left.getEncoder().setVelocityConversionFactor(Math.PI*0.152*(1/0.476) / 60);
        // right.getEncoder().setVelocityConversionFactor(Math.PI*0.152*(1/0.476) / 60);

        turningPID = new MiniPID(0.3/30, 0, 0.0) // need to actually find these values for the actual robot
        // .setI(0.05/200.0)
        .setOutputLimits(1)
        .setMaxIOutput(0.15)
        .setSetpointRange(30)
        ;


        ACCEL_DISTANCE = 0.5;
      break;

      case COMP:
        left.setInverted(true);
        leftA.setInverted(true);
        leftB.setInverted(true);
        right.setInverted(true);
        rightA.setInverted(true);
        rightB.setInverted(true);
        left.getEncoder().setPositionConversionFactor(Math.PI*6*Constants.INCHES_TO_METERS*(1/18.75));
        right.getEncoder().setPositionConversionFactor(Math.PI*6*Constants.INCHES_TO_METERS*(1/18.75));

        turningPID = new MiniPID(0.2/30, 0, 0.0) // need to actually find these values for the actual robot
        .setI(0.05/200.0)
        .setOutputLimits(0.2)
        .setMaxIOutput(0.15)
        .setSetpointRange(30)
        ;

        ACCEL_DISTANCE = 0.5;
      break;

      default:

    }

    //TODO: Must be removed if running PD controlled chassis, should be controlled better than commenting
    // drive = new DifferentialDrive(left, right);
    //Set up the follower motors
    // leftA.follow(left);
    // leftB.follow(left);
    // rightA.follow(right);
    // rightB.follow(right);

    //Try getting rid of follower setup and do direct writes to all motors
    drive = new DifferentialDrive(
      new SpeedControllerGroup(left,leftA,leftB),
      new SpeedControllerGroup(right,rightA,rightB)
    );


    //Configure the stall currents and limits
    int stallCurrent = 150/6;
    int freeCurrent = 240/6;
    int stallRPM = 5700/8;
    left.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);
    leftA.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);
    leftB.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);
    right.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);
    rightA.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);
    rightB.setSmartCurrentLimit(stallCurrent, freeCurrent, stallRPM);

    //Reset the encoders
    left.getEncoder().setPosition(0);
    right.getEncoder().setPosition(0);
    //Minimize motor timer overruns
    left.set(0);
    right.set(0);

    shift(Gear.LOW);

  }

  
  public void shift(Gear gear){
    shifter.set(gear.bool());
    shifterInverse.set(!gear.bool());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(getAverageDistance());
    // SmartDashboard.putNumber("Chassis/", value);
    // SmartDashboard.putBoolean("Chassis/shifter status", shifter.get());
    SmartDashboard.putNumber("Chassis/average distance", getAverageDistance());
    SmartDashboard.putNumber("Chassis/average velocity", getAverageVelocity());
    SmartDashboard.putNumber("Chassis/velocityLeft", left.getEncoder().getVelocity());
    SmartDashboard.putNumber("Chassis/velocityRight", right.getEncoder().getVelocity());
    SmartDashboard.putNumber("Chassis/inital compass heading", Constants.INITIAL_COMPASS_HEADING);


  }


  public CANSparkMax getLeftLeadMotor() {
    return left;
  }

  public CANSparkMax getRightLeadMotor() {
    return right;
  }

  public CANEncoder getLeftEncoder() {
    return left.getEncoder();
  }

  public CANEncoder getRightEncoder() {
    return right.getEncoder();
  }

  public double getAverageDistance() {
    return (LEFT_INVERSION*left.getEncoder().getPosition() + RIGHT_INVERSION*right.getEncoder().getPosition())/2.0;
  }

  public double getAverageVelocity() {
    return (LEFT_INVERSION*left.getEncoder().getVelocity() + RIGHT_INVERSION*right.getEncoder().getVelocity())/2.0;
  }

  public MiniPID getPID() {
    return turningPID;
  }

  public void setMotorIdleModes(IdleMode mode){
    left.setIdleMode(mode);
    right.setIdleMode(mode);
  }
}
