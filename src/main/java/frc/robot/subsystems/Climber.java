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
import com.stormbots.Lerp;
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
  MiniPID hookPID = new MiniPID(0.0,0,0); 

  SlewRateLimiter climbSlew = new SlewRateLimiter(5,0);

  /**Disables climber output for use with debug commands to rewind motors*/
  public boolean enable = false;
  
  public Climber() {
    System.out.print("Initializing Climber");
    switch(Constants.botName){
    case COMP:
      //Configure constants for the bot
      ARM_LENGTH_1 = 31.0;
      ARM_LENGTH_2 = 16.0;
    
      MAX_ARM_ANGLE = 89.0;
      MIN_ARM_ANGLE = 0.0;
    
      SPOOL_LENGTH = 25*2;
      CLIMBER_BASE_HEIGHT = 24.5;
      MAX_HEIGHT=72;//48+CLIMBER_BASE_HEIGHT+24-14.5; //82 is max height total
      MAX_HEIGHT=69;//safe empirical value

      //Configure motor setup
      armMotor.setInverted(true);
      armMotor.setIdleMode(IdleMode.kCoast);//Coast on bootup
      armMotor.setSmartCurrentLimit(80);
      armEncoder.setPositionConversionFactor(45/15.714277);
      armEncoder.setPositionConversionFactor(45/18.2);
      
      // armEncoder.setPositionConversionFactor(1);


      //INVERSION == FALSE IF SPOOLED SO LONG SIDE IS TOWARD SHOOTER
      spoolMotor.setInverted(false);
      spoolMotor.setIdleMode(IdleMode.kCoast);//Coast on bootup
      spoolMotor.setSmartCurrentLimit(20);
      spoolEncoder.setPositionConversionFactor(32/539.0);
  
      hookMotor.setInverted(true);
      hookMotor.setSmartCurrentLimit(10);//TODO:Set high for testing, make smaller
      hookMotor.setIdleMode(IdleMode.kCoast);//NOTE: Intentionally coast
      hookEncoder.setPositionConversionFactor(180/38.666*10);
      hookEncoder.setPositionConversionFactor(180.0/5.0);

      
      armEncoder.setPosition(0.0);
      spoolEncoder.setPosition(0.0);
      hookEncoder.setPosition(0.0);

      spoolPID = new MiniPID(1/9.0, 0, 0).setSetpointRange(12);
      climbPID = new MiniPID(1/12.0, 0.05/50.0, 0).setMaxIOutput(0.2);//.setSetpointRange(36);//May need higher P or I
      hookPID = new MiniPID(0.2/40.0, 0, 0);

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
      armMotor.setSmartCurrentLimit(40);
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
    //hookTargetAngle = hookEncoder.getPosition();
    hookTargetAngle = 115;
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
    // if(getArmHeight()<12+CLIMBER_BASE_HEIGHT)return;
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

  public void resetEncoders() {
    spoolEncoder.setPosition(0);
    armEncoder.setPosition(0);
    hookEncoder.setPosition(0);
  }

  // SlewRateLimiter targetSlew = new SlewRateLimiter(3); //Works, no sketchy spots. Verified after practice session
  SlewRateLimiter targetSlew = new SlewRateLimiter(5); //Works, tiny one sketchy spot, but should be ok
  // !!!!! SlewRateLimiter targetSlew = new SlewRateLimiter(8);//DON'T USE TOO FAST PROBABLY
  
  /*********** Periodic Stuff ********* */
  @Override
  public void periodic() {
    targetHeight = Clamp.clamp(targetHeight, CLIMBER_BASE_HEIGHT, MAX_HEIGHT);
    targetHeight = targetSlew.calculate(targetHeight);
    /* Height stuff */
    double spoolTargetHeight = MathUtil.clamp(targetHeight, CLIMBER_BASE_HEIGHT, CLIMBER_BASE_HEIGHT + SPOOL_LENGTH);
    // spoolTargetHeight = MathUtil.clamp(spoolTargetHeight, getArmHeight()+6, getArmHeight()+12);
    double spoolOutput = spoolPID.getOutput(getSpoolHeight(),spoolTargetHeight);
    
    //spoolOutput = Lerp.lerp(spoolOutput, 0.0, inMax, 0.0, MAX_HEIGHT);//??? why tho
    //attempt to prevent over-despooling
    // spoolTargetHeight = MathUtil.clamp(spoolTargetHeight, getArmHeight()-12,getArmHeight()+12);

    // setspoolheight()
    // climbheight(spool.getheight())
    double armTargetHeight = getSpoolHeight()-12;
    armTargetHeight = MathUtil.clamp(targetHeight, CLIMBER_BASE_HEIGHT, getSpoolHeight()-7);
    // armTargetHeight = targetHeight;

    // double armOutput = climbPID.getOutput(getArmHeight(), armTargetHeight);
    double armOutput = 0;
    armOutput += climbPID.getOutput(getArmHeight(), targetHeight);
    SmartDashboard.putNumber("armpid/outAfterPID", armOutput);
    SmartDashboard.putNumber("armpid/error", targetHeight- getArmHeight());
    //FB based close loop
    // armOutput += FB.fb(armTargetHeight, getArmHeight(), 0.09);
    armOutput += 0.1* Math.cos(Math.toRadians(armEncoder.getPosition()));
    SmartDashboard.putNumber("armpid/outAfterFF", armOutput);
    // armOutput = climbSlew.calculate(armOutput);
    armOutput = MathUtil.clamp(armOutput, -0.05, 1.0);
    SmartDashboard.putNumber("armpid/outputTotal", armOutput);

    double hookOutput = 0;
    // hookOutput = FB.fb(hookTargetAngle,hookEncoder.getPosition(),0.2); //janky and flops about
    hookOutput = hookPID.getOutput(hookEncoder.getPosition(), hookTargetAngle);
    hookOutput = Clamp.clamp(hookOutput, -0.4,0.4); //Not safety: Low output power is desirable for hook

    //TODO Remove/adjust safety clamps to appropriate values
    //spoolOutput = MathUtil.clamp(spoolOutput, -0.3, 0.3);
    // armOutput = MathUtil.clamp(armOutput, -0.05, 0.1);

    if(enable){
      //TODO : Enable and test climber
      spoolMotor.set(spoolOutput);
      armMotor.set(armOutput);
      hookMotor.set(hookOutput);
    }
    else{
      climbPID.reset();
      spoolPID.reset();

      spoolMotor.set(0);
      armMotor.set(0);
      hookMotor.set(0);
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
    SmartDashboard.putNumber("climb/armoutputRaw", armOutput);
    SmartDashboard.putNumber("climb/armoutput", armMotor.get());
    SmartDashboard.putNumber("climb/hookTarget", hookTargetAngle);
   }
}
