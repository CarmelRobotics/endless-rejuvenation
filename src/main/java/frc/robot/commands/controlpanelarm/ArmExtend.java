/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanelarm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelArm;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ArmExtend extends CommandBase {
  ControlPanelArm arm;
  public ArmExtend(ControlPanelArm a) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a);
    arm = a;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setExtender(DoubleSolenoid.Value.kReverse);
    System.out.println("extending");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("stopping extend command");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getExtender() == DoubleSolenoid.Value.kReverse);
  }
}
