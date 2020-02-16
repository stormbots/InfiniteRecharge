/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Passthrough.PassthroughState;

public class PassthroughLoadManually extends CommandBase {
  private Passthrough passthrough;

  /**
   * Creates a new PassthroughShoot.
   */
  public PassthroughLoadManually(Passthrough passthrough) {
    this.passthrough = passthrough;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(passthrough);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    passthrough.loadBall();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Periodic does all the work    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return passthrough.passthroughState == PassthroughState.IDLE;
  }

}
