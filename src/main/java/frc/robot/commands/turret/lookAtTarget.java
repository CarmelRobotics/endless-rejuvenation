/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class lookAtTarget extends CommandBase {
  /**
   * Creates a new fire.
   */
  private Turret t;
  private Vision v;
  public lookAtTarget(Turret t,Vision v) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.t = t;
    this.v = v;
    addRequirements(t);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("targeting limelight");
    if (v.getTY() < 4) {
        t.rotate(-3.0);
    }else if (v.getTY() > -4) {
        t.rotate(1);
    }else {
      System.out.println("not seeing target or already looking at it");
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    t.rotateStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
