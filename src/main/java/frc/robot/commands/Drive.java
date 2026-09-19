// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FuelConstants.LEFT_INTAKE_LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  CommandXboxController controller;

  private final SimpleMotorFeedforward m_xSpeedFeedforward = new SimpleMotorFeedforward(0.05, .65);
  private final SimpleMotorFeedforward m_zRotationFeedforward = new SimpleMotorFeedforward(0.05, .65);

  private final PIDController m_xSpeedPID = new PIDController(.4, 0.0, 0.0);
  private final PIDController m_zRotationPID = new PIDController(.4, 0, 0);
  

  public Drive(CANDriveSubsystem driveSystem, CommandXboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    controller = driverController;

    // set up pid tolerance
    m_xSpeedPID.setTolerance(.005);
    m_zRotationPID.setTolerance(.005);

    m_xSpeedPID.setIntegratorRange(-0.3, 0.3); 
    m_zRotationPID.setIntegratorRange(-0.3, 0.3); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // The Y axis of the controller is inverted so that pushing the
  // stick away from you (a negative value) drives the robot forwards (a positive
  // value). The X axis is scaled down so the rotation is more easily
  // controllable.
  @Override
  public void execute() {

    double targetVelocity = -controller.getLeftY();
    double targetRotationVelocity = controller.getRightX();

    double currentVelocity = (driveSubsystem.getLeftVelocity() + driveSubsystem.getRightVelocity()) / 2;
    double currentRotationVelocity = (driveSubsystem.getLeftVelocity() - driveSubsystem.getRightVelocity()) / 2;

    double xSpeedFeedforwardOutput = m_xSpeedFeedforward.calculate(targetVelocity);
    double zRotationFeedforwardOutput = m_zRotationFeedforward.calculate(targetRotationVelocity);

    double xSpeedPidOutput = m_xSpeedPID.calculate(currentVelocity, targetVelocity);
    double zRotationPidOutput = m_zRotationPID.calculate(currentRotationVelocity, targetRotationVelocity);

    double totalXSpeedOutput = xSpeedFeedforwardOutput + xSpeedPidOutput;
    double totalZRotationOutput = zRotationFeedforwardOutput + zRotationPidOutput;

    double xSpeedClampedOutput = MathUtil.clamp(totalXSpeedOutput, -1.0, 1.0);
    double zRotationClampedOutput = MathUtil.clamp(totalZRotationOutput, -1.0, 1.0);

    driveSubsystem.driveArcade(xSpeedClampedOutput, zRotationClampedOutput);


    SmartDashboard.putNumber("target velocity", targetVelocity);
    SmartDashboard.putNumber("target rotation velocity", targetRotationVelocity);

    SmartDashboard.putNumber("xSpeedClampedOutput", xSpeedClampedOutput);
    SmartDashboard.putNumber("zRotationClampedOutput", zRotationClampedOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
