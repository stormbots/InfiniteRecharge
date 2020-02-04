/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.stormbots.Lerp;
import com.stormbots.interp.SinCurve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDriveToHeading extends CommandBase {
  private final Chassis chassis;
  private DoubleSupplier encoder;
  private DoubleSupplier gyro;
  private double forwardDistance;
  private double targetHeading;
  private double initialPosition;
  private Lerp angleToPower = new Lerp(-180, 180, -0.5, 0.5);

  /**
   * Creates a new ChassisDriveManual.
   */
  public ChassisDriveToHeading(double forwardDistance, double targetHeading, DoubleSupplier encoder, DoubleSupplier gyro, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.encoder = encoder;
    this.gyro = gyro;
    this.forwardDistance = forwardDistance;
    this.targetHeading = targetHeading;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = encoder.getAsDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentAngle = (gyro.getAsDouble()%360) - 180;

    double forwardSpeed = SinCurve.ncurve(encoder.getAsDouble(), initialPosition, (initialPosition + forwardDistance), 0, 0.8);

    chassis.drive.arcadeDrive(
      forwardSpeed, 
      angleToPower.get(targetHeading - currentAngle)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    chassis.drive.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (encoder.getAsDouble() >= initialPosition+forwardDistance);
  }
}
