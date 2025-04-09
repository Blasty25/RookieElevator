// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ElevatorFeedforward ff =
      new ElevatorFeedforward(
          0.0, 0.0, 0.0, 0.0, 0.02);

  private Distance difference = Meters.zero();
  // KP is 110 ik but whatever!
  private ProfiledPIDController pid =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3.8, 4));

  public Elevator(ElevatorIO io) {
    this.io = io;
    pid.setTolerance(0.001);
  }

  public void setSetpoint(String setpoint) { // Setpoints are in Meters!
    if (setpoint.equals("STOW")) {
      setPosition(0, 0);
    }
    if (setpoint.equals("L1")) {
      setPosition(0.4, 0);
    }
    if (setpoint.equals("L2")) {
      setPosition(0.678, 0);
    }
    if (setpoint.equals("L3")) {
      setPosition(1.154, 0);
    }
    if (setpoint.equals("L4")) {
      setPosition(1.233, 0);
    }
  }

  public void setPosition(double position, double velocity) {
    pid.setGoal(new State(position, velocity));
  }

  public Command resetEncoder() {
    return Commands.runOnce(
            () -> {
              io.resetEncoder();
              System.out.println("Encoder");
            },
            this)
        .ignoringDisable(true);
  }

  public Command runSetpoint(String position) {
    return Commands.run(
        () -> {
          String setpoint = position;
          this.setSetpoint(setpoint);
        },
        this);
  }

  @Override
  public void periodic() {
    Logger.processInputs("Elevator", inputs);
    io.updateInputs(inputs);

    inputs.targetHeight = Meters.of(pid.getGoal().position);
    inputs.setpoint = pid.getGoal().position;

    double pidOutput = pid.calculate(inputs.currentHeight.in(Meters));
    double ffOutput = ff.calculateWithVelocities(inputs.velocity.in(MetersPerSecond), 0.27);

    Logger.recordOutput("Elevator/PIDOutput", pidOutput);
    Logger.recordOutput("Elevator/FFOutput", ffOutput * Math.signum(pidOutput));

    // TODO TEST only with PID CONTROL Remove ffOutput smth bugging not working!!!!!
    io.setVolts(pidOutput);

    difference = (inputs.targetHeight.minus(inputs.currentHeight));
    Logger.recordOutput("/Elevator/Difference", difference.in(Meters));
    Logger.recordOutput("Elevator/TargetHeight", inputs.targetHeight);
  }
}
