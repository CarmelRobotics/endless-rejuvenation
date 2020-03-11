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

public class AutoTest extends CommandBase {
  private DriveTrain drive;
  private double dist;
  public AutoTest(DriveTrain d, double dist) {
    addRequirements(d);
    drive = d;
    this.dist = dist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dist = drive.getEncRightDistance();
    System.out.println("distance is " + dist);
    drive.tankDrive(1.0,1.0);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.resetEncLeft();
    drive.resetEncRight();
    System.out.println("ending AutoTest");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return (drive.getEncLeftDistance() > dist);
  }
}
