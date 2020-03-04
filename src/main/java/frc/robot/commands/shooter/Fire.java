/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class Fire extends CommandBase {
  Intake intake;
  Turret turret;
  double ballVel = 0;
  private Vision vision;

  public Fire(Intake i, Turret t, Vision v) {
    addRequirements(i, t);
    intake = i;
    turret = t;
    vision = v;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleToTurnTo = turret.getAngleToTurnTo(ballVel, vision.getDistanceEstimation()/12, 32.2);
    turret.shoot();
    intake.agitate(0.5);
    intake.feed(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
