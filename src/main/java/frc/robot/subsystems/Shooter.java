/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stormbots.closedloop.MiniPID;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private final CANSparkMax shooterMotor = new CANSparkMax(11 ,MotorType.kBrushless);
  private final CANSparkMax feederMotor = new CANSparkMax(10 ,MotorType.kBrushless);
  private final CANEncoder encoder = new CANEncoder(shooterMotor);


  MiniPID feedForwardPID = new MiniPID(0,0,0,1/(5700.0*2)*1.1);
  MiniPID errorPID = new MiniPID(1/5700.0*1.2,0,0).setOutputLimits(-0.05, 0.5);
  private final SlewRateLimiter feedForwardSlew = new SlewRateLimiter( 1/2.0 ,0);

  double targetRPM = 0;

  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.154,0.0425,0.0202);

  public Shooter() {
    switch(Constants.botName){
      case COMP:
        feederMotor.setInverted(true);
      break;

      case PRACTICE:
      default:
        feederMotor.setInverted(true);
      break;
    }
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotor.setOpenLoopRampRate(0.1);

    encoder.setPosition(0);
    encoder.setPositionConversionFactor(2);
    encoder.setVelocityConversionFactor(2);

    if(!SmartDashboard.containsKey("shooter/RMPDebugSet"))SmartDashboard.putNumber("shooter/RMPDebugSet", 1000);
  }

  public void reset() {
    feedForwardPID.reset();
    errorPID.reset();
  }

  public void setRPM(double rpm){
    this.targetRPM = rpm;
  }

  private void runClosedLoop() {
    double feedForwardOutput = feedForwardPID.getOutput(encoder.getVelocity(), targetRPM) ;
    double errorOutput = errorPID.getOutput(encoder.getVelocity(), targetRPM);


    // double feedForwardOutput = feedForward.calculate(targetRPM,0.0202);//todo: Accelleration

    if (encoder.getVelocity() < targetRPM*.5) {
      errorOutput = 0.0;
    }

    feedForwardOutput *=2; 
    //TODO Feeder is geared to 1/10th the speed of shooter. Just run it as fast as possible now
    // and we'll be moving it to the Passthrough in a bit.

    feedForwardOutput*=0.5;// Rely more on error feedback because bad calculations

    feedForwardOutput = feedForwardSlew.calculate(feedForwardOutput);
    shooterMotor.set(feedForwardOutput + errorOutput);
    feederMotor.set(feedForwardOutput + errorOutput);

    SmartDashboard.putNumber("shooter/contribFF", feedForwardOutput);
    SmartDashboard.putNumber("shooter/contribPID", errorOutput);

  }

  //public void setMotorSpeed(double speed) {
  //  shooterMotor.set(speed);
  //}


  @Override
  public void periodic() {

    runClosedLoop();

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter/RPM", encoder.getVelocity());
    SmartDashboard.putNumber("shooter/amps", shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/positons", encoder.getPosition());
    SmartDashboard.putNumber("shooter/appliedOutput", shooterMotor.getAppliedOutput());
    SmartDashboard.putNumber("shooter/outputCurrent", shooterMotor.getOutputCurrent());
  }
}
