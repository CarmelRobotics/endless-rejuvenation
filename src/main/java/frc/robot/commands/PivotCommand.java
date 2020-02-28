/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class PivotCommand extends CommandBase {
  Vision vision;
  double P = 1.8;
  double I = 0.0;
  double D = 0.0;
  double rcw = 0.0;
  DriveTrain d;
  public PivotCommand(Vision v, DriveTrain d) {
    this.vision = v;
    this.d = d;
  }
  // private double convert(double percent) {

  // }%
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  void turnAtSpeed(double speed) {
    d.tankDrive(-speed, speed);
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    try {
      turnAtSpeed(vision.pivotToTarget(0.5,0.3,0.1));
    } catch (NullPointerException e) {
      System.out.println("Caught Exception");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
      System.out.print("");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
