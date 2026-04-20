// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class shootcommand extends Command {
  /** Creates a new shootcommand. */
  private final double shootercommand;
  private final double setShooterSpeed;
  private final double shooter2Speed;
  
  private final double intakeFlueSpeed;
  public shootcommand(double setShooterSpeed, double shooter2Speed, Double shootercommand, double intakeFlueSpeed) {
    this.setShooterSpeed = setShooterSpeed;
    this.shooter2Speed = shooter2Speed;
    this.shootercommand = shootercommand;
    this.intakeFlueSpeed = intakeFlueSpeed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
