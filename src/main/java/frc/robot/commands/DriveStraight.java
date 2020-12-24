/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveStraight extends PIDCommand {
  /**
   * Creates a new DriveStraight.
   */

  public DriveStraight(Drivetrain drive, double distSetPt) {
    super(
        // The controller that the command will use
        new PIDController(Constants.PIDConstants.drivekP, Constants.PIDConstants.driveKI, Constants.PIDConstants.driveKD),
        // This should return the measurement
        () -> (drive.getLeftEncoderDistance()+drive.getRightEncoderDistance()) / 2,
        // This should return the setpoint (can also be a constant)
        () -> distSetPt,
        // This uses the output
        output -> {
            drive.driveStraight(output, Constants.PIDConstants.ksetPoint);
          }
        );
        addRequirements(drive);
        drive.resetGryo();
        drive.resetDrivetrainEncoders();
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
