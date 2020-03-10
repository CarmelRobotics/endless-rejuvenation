/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class PivotCommand extends CommandBase {
  Vision vision;
  private static double P = 1.8;
  private static double I = 0.0;
  private static double D = 0.0;
  double rcw = 0.0;
  DriveTrain d;
  Turret turret;
  public PivotCommand(Vision v, DriveTrain d, Turret t) {
    this.vision = v;
    this.d = d;
    this.turret = t;
    turret.resetNAVX();
  }
  // private double convert(double percent) {

  // }%
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    turret.resetNAVX();
  }

  void turnAtSpeed(double speed) {
    d.tankDrive(speed, -speed);
  }
  boolean turnToAngle(double angle, double error) {
    double angleAt = Math.abs(turret.getNAVXValue());
    // double angleDiff = Math.abs(turret.getNAVXAngle())-angle;
    double diff = Math.abs(angleAt-angle);
    // double maxSpeed = angle/100;
    double speed = 0.45;

    if (diff < error + 10) {
      speed = 0.35;
    }
    if (diff < error) {
      turret.rotateStop();
      return true;
    }

    if (angleAt < angle) {
      // turret.rotate(speed-((diff)/300));
      turret.rotate(speed);
      return false;
    } else if (angleAt > angle) {
      // turret.rotate(-(speed)-(diff/200));
      turret.rotate(-speed);
      return false;
    } else {
      turret.rotateStop();
      return true;
    }
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double angleToTurnTo = turret.solveForDegrees(vision.getDistanceEstimation());
    SmartDashboard.putNumber("TURRET ANGLE", turret.getNAVXAngle());
    SmartDashboard.putNumber("ANGLE TO TURN TO", turret.solveForDegrees(vision.getDistanceEstimation()/12));
    turnAtSpeed(vision.pivotToTarget(0.5,0.3,0.1));

    // System.out.println("Turret angle is " + turret.getNAVXAngle());
    try {
      if (turnToAngle(angleToTurnTo, 1) == true || turret.isOutsideRange() == true) {
        turret.rotateStop();
      }
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
      turret.rotateStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
}
