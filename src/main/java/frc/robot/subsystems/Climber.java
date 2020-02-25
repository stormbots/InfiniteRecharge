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
import com.stormbots.Clamp;
import com.stormbots.closedloop.FB;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  CANSparkMax armMotor = new CANSparkMax(12, MotorType.kBrushless);
  public CANSparkMax spoolMotor = new CANSparkMax(14, MotorType.kBrushless);
  public CANSparkMax hookMotor = new CANSparkMax(13, MotorType.kBrushless);

  CANEncoder armEncoder = new CANEncoder(armMotor);
  CANEncoder spoolEncoder = new CANEncoder(spoolMotor);
  CANEncoder hookEncoder = new CANEncoder(hookMotor);

  //Physical parameters of robot
  double ARM_LENGTH_1;
  double ARM_LENGTH_2;
  double MAX_ARM_ANGLE;
  double MIN_ARM_ANGLE;
  double SPOOL_LENGTH; // TODO: find new spool length
  public double CLIMBER_BASE_HEIGHT;
  public double MAX_HEIGHT;

  double targetHeight;
  private double hookTargetAngle;

  MiniPID spoolPID = new MiniPID(0.0, 0, 0);
  MiniPID climbPID = new MiniPID(0.0,0,0);
 
  SlewRateLimiter climbSlew = new SlewRateLimiter(1,0);

  /**Disables climber output for use with debug commands to rewind motors*/
  public boolean disable = false;
  
  public Climber() {
    System.out.print("Initializing Climber");
    switch(Constants.botName){
    case COMP:
      //Configure constants for the bot
      ARM_LENGTH_1 = 32.75;
      ARM_LENGTH_2 = 28;
    
      MAX_ARM_ANGLE = 89.0;
      MIN_ARM_ANGLE = 0.0;
    
      SPOOL_LENGTH = 38.0;
      CLIMBER_BASE_HEIGHT = 24.5;
      MAX_HEIGHT=SPOOL_LENGTH+CLIMBER_BASE_HEIGHT;

      //Configure motor setup
      armMotor.setInverted(true);
      armMotor.setIdleMode(IdleMode.kCoast);//Coast on bootup
      armMotor.setSmartCurrentLimit(20);
      armEncoder.setPositionConversionFactor(90/59.88);

      spoolMotor.setInverted(false);
      spoolMotor.setIdleMode(IdleMode.kCoast);//Coast on bootup
      spoolMotor.setSmartCurrentLimit(20);
      spoolEncoder.setPositionConversionFactor(32/539.0);
  
      hookMotor.setInverted(true);
      hookMotor.setSmartCurrentLimit(2);
      hookMotor.setIdleMode(IdleMode.kCoast);//NOTE: Intentionally coast
      hookEncoder.setPositionConversionFactor(180/38.666);
      
      armEncoder.setPosition(0.0);
      //spoolEncoder.setPosition(0.0);
      hookEncoder.setPosition(0.0);

      spoolPID = new MiniPID(1/12.0, 0, 0).setSetpoint(12);
      climbPID = new MiniPID(1/12.0, 0, 0).setSetpoint(12);
    break;
    case PRACTICE:
    //fallthrough to default
    default:
      //Configure constants for the bot
      ARM_LENGTH_1 = 32.5;
      ARM_LENGTH_2 = 22.5;
    
      MAX_ARM_ANGLE = 89.0;
      MIN_ARM_ANGLE = 0.0;
    
      SPOOL_LENGTH = 38.0;
      CLIMBER_BASE_HEIGHT = 24.5;
      MAX_HEIGHT=SPOOL_LENGTH+CLIMBER_BASE_HEIGHT;

      //Configure motors and scaling
      armMotor.setInverted(true);
      armMotor.setIdleMode(IdleMode.kBrake);
      armMotor.setSmartCurrentLimit(20);
      armEncoder.setPositionConversionFactor(90/65.5);

      spoolMotor.setInverted(true);
      spoolMotor.setIdleMode(IdleMode.kBrake);
      spoolMotor.setSmartCurrentLimit(20);
      spoolEncoder.setPositionConversionFactor(38/1355.5);
      
      hookMotor.setSmartCurrentLimit(2);
      hookMotor.setIdleMode(IdleMode.kCoast);
      hookEncoder.setPositionConversionFactor(1);
      
      armEncoder.setPosition(0.0);
      spoolEncoder.setPosition(0.0);
    break;
    }

    /*NOTE: Hook current should be the lowest usable value, 
    since we don't want it doing much. It should deploy, and then 
    be allowed to be back-driven by the rest of the climber system
    */
    hookTargetAngle = hookEncoder.getPosition();
    targetHeight = getSpoolHeight();


    //Throw debug functions on SmartDashboard
    SmartDashboard.putData("climb/hookGrab",new InstantCommand(()->setHookAngle(180)));
    SmartDashboard.putData("climb/hookRetract",new InstantCommand(()->setHookAngle(0)));
  }
  
  /* Main functions */
  
  public void setHeight(double targetHeight){
    this.targetHeight = targetHeight;
  }

  public double getHeight(){
    return getSpoolHeight();
  }

  public void setHookAngle(double angle){
    //Don't allow the hook to deploy if the climber's down
    if(getArmHeight()<12+CLIMBER_BASE_HEIGHT)return;
    this.hookTargetAngle=angle;
  }

  public void setMotorIdleModes(IdleMode mode){
    armMotor.setIdleMode(mode);
    spoolMotor.setIdleMode(mode);
  }

  /* Helper Functions */
  private double getSpoolHeight(){
    double currentSpoolHeight = spoolEncoder.getPosition()+CLIMBER_BASE_HEIGHT;
    return currentSpoolHeight; 
  }


  private double getArmHeight(){
    double theta = armEncoder.getPosition();
    theta = Math.toRadians(theta);

    return (Math.sin(theta) * ARM_LENGTH_1) + (Math.sin(theta) * ARM_LENGTH_2) + CLIMBER_BASE_HEIGHT;
  }


  /*********** Periodic Stuff ********* */
  @Override
  public void periodic() {
    targetHeight = Clamp.clamp(targetHeight, CLIMBER_BASE_HEIGHT, MAX_HEIGHT);
    /* Height stuff */
    double spoolOutput = spoolPID.getOutput(getSpoolHeight(), targetHeight);
   
    // setspoolheight()
    // climbheight(spool.getheight())
    double armTargetHeight = getSpoolHeight();

    double armOutput = climbPID.getOutput(getArmHeight(), armTargetHeight);
    //FB based close loop
    armTargetHeight = MathUtil.clamp(armTargetHeight,0,80);
    armOutput = FB.fb(armTargetHeight, armEncoder.getPosition(), 0.07);
    armOutput = climbSlew.calculate(armOutput);

    double hookOutput = 0;
    hookOutput = FB.fb(hookTargetAngle,hookEncoder.getPosition(),0.04);
    hookOutput = Clamp.clamp(hookOutput, -0.3,0.3); //Not safety: Low output power is desirable for hook

    //TODO Remove/adjust safety clamps to appropriate values
    spoolOutput = MathUtil.clamp(spoolOutput, -0.3, 0.3);
    armOutput = MathUtil.clamp(armOutput, -0.1, 0.1);

    if(!disable){
      //TODO : Enable and test climber
      //spoolMotor.set(spoolOutput);
      //armMotor.set(armOutput);
      //hookMotor.set(hookOutput);
    }

    //Update SmartDashbaord
    SmartDashboard.putNumber("climb/targetHeight", targetHeight);

    SmartDashboard.putNumber("climb/armEncoder", armEncoder.getPosition());
    SmartDashboard.putNumber("climb/armHeight", getArmHeight());
    SmartDashboard.putNumber("climb/armAmpsOutput", armMotor.getOutputCurrent());

    SmartDashboard.putNumber("climb/spoolEncoder", spoolEncoder.getPosition());
    SmartDashboard.putNumber("climb/spoolHeight", getSpoolHeight());

    SmartDashboard.putNumber("climb/hookAngle", hookEncoder.getPosition());
    SmartDashboard.putNumber("climb/hookAmps", hookMotor.getOutputCurrent());
    SmartDashboard.putNumber("climb/spooloutput", spoolMotor.get());
    SmartDashboard.putNumber("climb/armoutput", armMotor.get());


   }
}
