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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final CANSparkMax shooterMotor = new CANSparkMax(1 ,MotorType.kBrushless);
  private final CANEncoder encoder = new CANEncoder(shooterMotor);

  MiniPID feedForwardPID = new MiniPID(0,0,0,1/5700.0);
  MiniPID errorPID = new MiniPID(1/5700.0,0,0);

  double targetRPM = 0;

  public Shooter() {
    shooterMotor.setIdleMode(IdleMode.kCoast);
    feedForwardPID.setSetpointRange(2000/2.0);
    errorPID.setOutputLimits(0.0, 0.5);
  }

  public void reset() {
    feedForwardPID.reset();
    errorPID.reset();
  }

  public void setRPM(double rpm){
    this.targetRPM = rpm;
  }

  private void runClosedLoop() {
    double feedForwardOutput = feedForwardPID.getOutput(encoder.getVelocity(), targetRPM);
    double errorOutput = errorPID.getOutput(encoder.getVelocity(), targetRPM);

    if (encoder.getVelocity() < targetRPM*.5) {
      errorOutput = 0.0;
    }
    

    shooterMotor.set(feedForwardOutput + errorOutput);
  }

  public void setMotorSpeed(double speed) {
    shooterMotor.set(speed);
  }


  @Override
  public void periodic() {

    runClosedLoop();

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RPM", encoder.getVelocity());
    SmartDashboard.putNumber("Current", shooterMotor.getOutputCurrent());
  }
}
