/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.ThreeWaySwitchConstants;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.shooter.Fire;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Auto extends SequentialCommandGroup {
  /**
   * Creates a new Auto.
   */
  public Auto(DriveTrain d, Intake i, Turret t, Vision v) {
    if (ThreeWaySwitchConstants.SWITCH_1 == null ||
        ThreeWaySwitchConstants.SWITCH_3 == null) {
          System.out.println("NULL SWITCH DETECTED: USING AUTO 1 AS DEFAULT");
          addCommands(new Auto1(d,i,t,v));
    }else {
      if (ThreeWaySwitchConstants.SWITCH_1.get() == true) {
        addCommands(new Auto1(d,i,t,v));
        System.out.println("USING AUTO 1");
      } else if (ThreeWaySwitchConstants.SWITCH_3.get() == true) {
        addCommands(new Auto3(d,i,t,v));
        System.out.println("USING AUTO 3");
      } else {
        addCommands(new Auto2(d,i,t,v));
        System.out.println("USING AUTO 2");
      }
    }
    
    }
}
