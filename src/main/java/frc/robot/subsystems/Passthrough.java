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
import com.stormbots.Clamp;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Passthrough extends SubsystemBase {

  private CANSparkMax motor = new CANSparkMax(9,MotorType.kBrushless);
  public CANEncoder encoder = new CANEncoder(motor);
  

  MiniPID pid;

  
  final double BALLLENGTH = 7; //inches
  double PASSTHROUGHLENGTH = 27; //Placeholder Number, Replace Later  
  double positionOfFirstBall = 0; //positon ahead of the first ball
  double positionOfLastBall = 0; //position behind the last ball
  double setpoint=0; //targetposition
  int numberOfBalls = 0;  //The robot can have 3 balls at the start of the match

  private final boolean BLOCKED = false; // TODO find correct value
  private final boolean NOTBLOCKED = !BLOCKED; // TODO find correct value
  private DigitalInput intakeSensor = new DigitalInput(1); //sensor on the begining of passthrough
  private DigitalInput readySensor = new DigitalInput(0); //sensor that's outside the passthrough that detects if a ball is ready to intake 
  private DigitalInput shootSensor = new DigitalInput(3); // sensor closest to shooter
  private boolean shootSensorLastReading = NOTBLOCKED;
  private boolean intakeSensorLastReading = NOTBLOCKED;
  private boolean readySensorLastReading = NOTBLOCKED;

  public enum PassthroughState{IDLE,LOADING,SHOOTING,EJECTING,DISABLED};
  public PassthroughState passthroughState = PassthroughState.IDLE;
  /**
   * Creates a new Passthrough.
   */
  public Passthrough() {
    //Throw debug commands on the dashboard
    SmartDashboard.putData("pt/LoadBall",new InstantCommand(()->loadBall()));
    SmartDashboard.putData("pt/Shoot",new InstantCommand(()->shoot()));
    SmartDashboard.putData("pt/Eject",new InstantCommand(()->eject()));
    switch(Constants.botName){
      case COMP:
      //fallthrough until otherwise known
    case PRACTICE:
      encoder.setPositionConversionFactor(21.25/42.2); //TODO: make sure this ratio is right, flip numerator and denominator if not right
      pid = new MiniPID(1/3.0,0,0)
      .setOutputLimits(0.3);
      ;
    break;
    case TABI://fallthrough
    default:
    encoder.setPositionConversionFactor(21.25/4.0);//set for the wheel on tabi
    pid = new MiniPID(0.01,0,0).setOutputLimits(0.12);
    }

    motor.setInverted(false);
    motor.setSmartCurrentLimit(20,30,30); //TODO Test current constraints
    
    //reset the system's positioning
    encoder.setPosition(0);
    positionOfFirstBall=0;
    positionOfLastBall=0;
    pid.setSetpoint(encoder.getPosition());

    //avoid sensor edge startup glitches
    shootSensorLastReading = shootSensor.get();
    intakeSensorLastReading = intakeSensor.get();
    readySensorLastReading = readySensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentPosition = encoder.getPosition();    

    
    boolean shootSensorReading = shootSensor.get();
    boolean intakeSensorReading = intakeSensor.get();
    boolean readySensorReading = readySensor.get();

    //Automatically subtract balls that leave through the shooter
    if(positionOfFirstBall > (encoder.getPosition() + PASSTHROUGHLENGTH)){
      positionOfFirstBall = positionOfFirstBall - BALLLENGTH;
      numberOfBalls -= numberOfBalls>0 ? 1 : 0;
    };
    
    if(encoder.getVelocity() < 0){ //Balls moving toward shooter
      // if(shootSensorLastReading == NOTBLOCKED &&  shootSensorReading == BLOCKED ){
      //   double oldfirstball = positionOfFirstBall;
      //   positionOfFirstBall = encoder.getDistance() + PASSTHROUGHLENGTH;
      //   if(setpoint == oldfirstball) setpoint = positionOfFirstBall;
      // } 
      if(intakeSensorLastReading == BLOCKED && intakeSensorReading == NOTBLOCKED) {
        double oldlastball = positionOfLastBall;
        positionOfLastBall = encoder.getPosition();
        if(setpoint == oldlastball) setpoint = positionOfLastBall;
      }
    }else{ // balls moving away from shooter
      // if(shootSensorLastReading == BLOCKED && shootSensorReading == NOTBLOCKED) {
      //   double oldfirstball = positionOfFirstBall;
      //   positionOfFirstBall = encoder.getDistance() + PASSTHROUGHLENGTH;
      //   if(setpoint == oldfirstball) setpoint = positionOfFirstBall;
      // }
      if(intakeSensorLastReading == NOTBLOCKED && intakeSensorReading == BLOCKED) {
        //don't need to go
        double oldlastball = positionOfLastBall;
        positionOfLastBall = encoder.getPosition();
        if(setpoint == oldlastball) setpoint = positionOfLastBall;
      }
    }

    switch(passthroughState){
      case IDLE: 
        if(readySensorLastReading != readySensorReading && DriverStation.getInstance().isEnabled()){
          loadBall();
        }
      break;
      case LOADING:
        if(isOnTarget(1)){
          passthroughState = PassthroughState.IDLE; 
        }
      break;
      case SHOOTING:
        if(isOnTarget(1)){
          passthroughState = PassthroughState.IDLE; 
        }
      break;
      case EJECTING:
      if(isOnTarget(1)){
        passthroughState = PassthroughState.IDLE; 
      }
      break;
      default:
      //Do nothing?
    }

    //Actuate motors!
    double output = 0;
    switch(passthroughState){
      case DISABLED:
        motor.set(0);
      break;
      default:
        output = pid.getOutput(currentPosition, setpoint);
        motor.set(output);
    }

    // Save our edge states
    shootSensorLastReading = shootSensorReading;
    intakeSensorLastReading = intakeSensorReading;
    readySensorLastReading = readySensorReading;

    // Print things to dashboard
    SmartDashboard.putNumber("pt/posTarget", setpoint);
    SmartDashboard.putNumber("pt/posCurrent", currentPosition);
    SmartDashboard.putString("pt/state", passthroughState.toString());
    SmartDashboard.putNumber("pt/motorOutput", output);
    SmartDashboard.putNumber("pt/posLastBall", positionOfLastBall);
    SmartDashboard.putNumber("pt/posFirstBall", positionOfFirstBall);
    SmartDashboard.putBoolean("pt/sensorReady", isReadySensorBlocked());
    SmartDashboard.putNumber("pt/numBalls", numberOfBalls);
    // SmartDashboard.putBoolean("pt/sensorBackOfQueue", intakeSensor.get());
    // SmartDashboard.putBoolean("pt/sensorShooter", shootSensor.get());

    SmartDashboard.putNumber("pt/output", motor.getOutputCurrent());
    SmartDashboard.putBoolean("pt/isOnTarget", isOnTarget(1));
  }

  public void loadBall() {
    if(numberOfBalls >= 4) return;
    if(passthroughState == PassthroughState.LOADING)return;
    passthroughState = PassthroughState.LOADING;

    setpoint = encoder.getPosition() - BALLLENGTH;
    positionOfLastBall = setpoint;
    if(numberOfBalls == 0){
      positionOfFirstBall = positionOfLastBall+BALLLENGTH;
    }
    numberOfBalls += 1;
  }

  public void prepareForLoading(){
    setpoint = positionOfLastBall;
  }

  //TODO: This function is probably not needed, but does work
  // public void prepareForShooting(){
  //   setpoint = positionOfFirstBall - PASSTHROUGHLENGTH;
  // } 

  /** Dump all the balls out the front */
  public void shoot(){
    if(passthroughState == PassthroughState.SHOOTING)return;
    passthroughState = PassthroughState.SHOOTING;
    setpoint = encoder.getPosition() - PASSTHROUGHLENGTH - 2*BALLLENGTH;
  }

  /** Dump all balls out the intake */
  public void eject() {   
    if(passthroughState == PassthroughState.EJECTING)return;
    passthroughState = PassthroughState.EJECTING;
    setpoint = encoder.getPosition() + PASSTHROUGHLENGTH + 2*BALLLENGTH;
    numberOfBalls = 0;
  }

  public boolean isReadySensorBlocked(){
    return readySensor.get()==BLOCKED;
  }

  //
  public void reset(){
    passthroughState = PassthroughState.IDLE;
    setpoint = encoder.getPosition(); 
  }

  public double getBallCount(){
    return numberOfBalls;
  }

  public boolean isOnTarget(double tolerance){
    return Clamp.bounded(encoder.getPosition(),setpoint-tolerance,setpoint+tolerance);
  }
}
