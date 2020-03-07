/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hanger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hanger;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;

public class ReachHook extends CommandBase {
  private Hanger hanger;
  boolean hooked;
  public ReachHook(Hanger h) {
    hanger = h;
    hooked = false;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (!hooked)
      hanger.climb(1.0);
    else
      hanger.climb(-1.0);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      hanger.hook(DoubleSolenoid.Value.kForward);
      hanger.climb(0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return hanger.getEncoder() > 0; //replace
  }
}
