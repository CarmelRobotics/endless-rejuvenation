/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoMove extends CommandBase {
  private DriveTrain drive;
  private double encDiff;
  private double moveDistance;
  public AutoMove(DriveTrain d, double distance) {
    addRequirements(d);
    drive = d;
    moveDistance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    encDiff = drive.getEncLeftDistance() - drive.getEncRightDistance();
    if (encDiff > 0.004) { //at wheel dist = 22.5 in, assuming no skid, this allows 0.1 degree drift before correcting (two encoder steps)
      drive.tankDrive(0.9, 1.0); //CALIBRATE
    } else if (encDiff < -0.004) {
      drive.tankDrive(1.0, 0.9); //CALIBRATE
    } else {
      drive.tankDrive(1.0, 1.0);
    }
    
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.resetEncLeft();
    drive.resetEncRight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return (drive.getEncLeftDistance() > moveDistance && drive.getEncRightDistance() > moveDistance); //CALIBRATE
  }
}
