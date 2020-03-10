/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class Rotate_Up extends CommandBase {
  /**
   * Creates a new fire.
   */
  private Turret t;
  public Rotate_Up(Turret t) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.t = t;
    addRequirements(t);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    t.rotate(0.45);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    t.rotateStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
