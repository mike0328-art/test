// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

import frc.robot.Constants.SwerveConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final SparkFlex shooterMotor;
  private final SparkFlex shooterMotor2;
  private final TalonFX turningangle;
  private final TalonFX intakeFlue;

  private final SparkFlexConfig config;
  private final SparkBaseConfig config2;
  private final TalonFXConfiguration turningConfig;
  private final TalonFXConfiguration intakeConfig;

  private PIDController intakController;
  public Shooter() {
    shooterMotor = new SparkFlex(frc.robot.Constants.SwerveConstants.ShooterConstants.shooterMotorID, MotorType.kBrushless);
    shooterMotor2 = new SparkFlex(frc.robot.Constants.SwerveConstants.ShooterConstants.shooterMotor2ID, MotorType.kBrushless);
    turningangle = new TalonFX(frc.robot.Constants.SwerveConstants.ShooterConstants.turningAngleID);
    intakeFlue = new TalonFX(frc.robot.Constants.SwerveConstants.ShooterConstants.intakeFlueID);

    config = new SparkFlexConfig();
    config2 = new SparkFlexConfig();
    turningConfig = new TalonFXConfiguration();
    intakeConfig = new TalonFXConfiguration();
    intakController = new PIDController(0.1, 0, 0);
    intakController.enableContinuousInput(ModuleConstants.pidRangeMin, ModuleConstants.pidRangeMax);

    config.smartCurrentLimit(60);
    config2.smartCurrentLimit(60);

    shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    
  }

  @Override
  
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopShooter() {
    shooterMotor.set(0);
    shooterMotor2.set(0);
  }
}
