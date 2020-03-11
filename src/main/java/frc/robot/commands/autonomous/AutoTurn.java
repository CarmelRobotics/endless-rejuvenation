/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoTurn extends CommandBase {
  private DriveTrain drive;
  double angle, error, speed;
  boolean done;
  public AutoTurn(DriveTrain d, double angle, double error, double speed) {
    addRequirements(d);
    drive = d;
    this.angle = angle;
    this.error = error;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("running AutoTurn");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (done == false) {
      double navXAngle = Constants.NAVXConstants.NAVX.getYaw();
      if (navXAngle < angle - error) {
        drive.tankDrive(-speed, speed);
        System.out.println("turning right");
      }else if (navXAngle > angle + error){
        drive.tankDrive(speed, -speed);
        System.out.println("turning left");
      }else{
        done = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoTurn finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
