// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.MAShuffleboard.pidControllerGainSupplier;
import com.ma5951.utils.subsystem.DefaultControlSubsystemInSubsystemControl;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements 
  DefaultControlSubsystemInSubsystemControl{
  /** Creates a new Elevator. */
  private CANSparkMax motor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;
  private MAShuffleboard board;
  private pidControllerGainSupplier pidSupplier;
  private double setPoint;
  private static Elevator instance;

  private Elevator() {
    motor = new CANSparkMax(ElevatorConstance.motorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(
      ElevatorConstance.positionConversionFactor
    );
    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);
    pidController.setP(ElevatorConstance.kP);
    pidController.setI(ElevatorConstance.kI);
    pidController.setD(ElevatorConstance.kD);

    board = new MAShuffleboard("Elevator");
    pidSupplier = board.getPidControllerGainSupplier(
      ElevatorConstance.kP, ElevatorConstance.kI, ElevatorConstance.kD);
  }

  public double getFeed() {
    return ElevatorConstance.kG;
  }

  @Override
  public void calculate(double setPoint) {
    pidController.setReference(setPoint, ControlType.kPosition,
      0, getFeed(), ArbFFUnits.kPercentOut);
  }

  @Override
  public boolean atPoint() {
    return Math.abs(encoder.getPosition() - setPoint) <=
     ElevatorConstance.positionConversionFactor;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.set(voltage / 12);
  }

  @Override
  public boolean canMove() {
    return setPoint > ElevatorConstance.minPose
      && setPoint < ElevatorConstance.maxPose;
  }

  @Override
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  @Override
  public double getSetPoint() {
    return setPoint;
  }

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }
    return instance;
  }

  @Override
  public void periodic() {
    board.addNum("pose", encoder.getPosition());
    pidController.setP(pidSupplier.getKP());
    pidController.setI(pidController.getI());
    pidController.setD(pidSupplier.getKD());
  }
}
