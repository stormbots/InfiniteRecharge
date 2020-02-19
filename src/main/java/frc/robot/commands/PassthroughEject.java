/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;

public class PassthroughEject extends CommandBase {
  private Passthrough passthrough;
  private Intake intake;

  /**
   * Creates a new PassthroughShoot.
   */
  public PassthroughEject(Passthrough passthrough, Intake intake) {
    this.passthrough = passthrough;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(passthrough);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    passthrough.eject();
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.intakeReverse();
    //Periodic does all the work    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return passthrough.isOnTarget(4);
  }

}
