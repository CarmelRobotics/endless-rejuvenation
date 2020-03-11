/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanelarm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelArm;

public class ControlPanelRotCtrl extends CommandBase {
  /**
   * Creates a new ControlPanelStageOne.
   */
  private ControlPanelArm arm;
  private char initialColor;
  private char lastColor = '?';
  private char currentColor;
  private int halfRotations;
  private int targetRotations;
  private int totalChanges;
  public ControlPanelRotCtrl(ControlPanelArm cpa, int rot) {
    addRequirements(cpa);
    arm = cpa;
    targetRotations = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("run Da Command");
    initialColor = arm.detectColor();
    halfRotations = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (initialColor == '?')
      initialColor = arm.detectColor();
    arm.spin(0.22);
    currentColor = arm.detectColor();
    if (currentColor != '?') {
      if (currentColor != lastColor) {
        totalChanges++;
        System.out.println("color change! from " + lastColor + " to: " + currentColor + ". " + totalChanges);
        if (currentColor == initialColor && totalChanges != 0) {
          halfRotations++;
          System.out.println("one half rotation detected! rotations: " + halfRotations);
        }
      }
      lastColor = currentColor;
    }else{
      System.out.println("No color detected!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopSpin();
    System.out.println("stopping");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return halfRotations >= targetRotations;
  }
}
