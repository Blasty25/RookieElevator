// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;

/* This is the explanation on what to do here
 * Make a instance of the SparkMax Config Class like so: SparkMaxConfig config = new SparkMaxConfig;
 * After that in the Constructor(The constructor runs once in each class(often on intialization) public ElevatorIOReal(){} is a constructor)
 * You need to do config.idleMode(coast); This will make the motors move easier.
 * You can look throught the different configs to see what is the best config for a elevator
 * If you have any questions about any of the configs ask about them in the programming channel.
 */

public class ElevatorIOReal implements ElevatorIO {

  public RelativeEncoder encoder;
  public ElevatorIOReal() {

    // config
    //     .encoder
    //     .positionConversionFactor(ElevatorConstants.positionConversionFactor)
    //     .velocityConversionFactor(ElevatorConstants.positionConversionFactor / 60);


    // encoder = leftSparky.getEncoder();
    // encoder.setPosition(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // inputs.currentHeight = Meters.ofBaseUnits(encoder.getPosition());

    // inputs.leftCurrent = Amps.of(leftSparky.getOutputCurrent());
    // inputs.rightCurrent = Amps.of(rightSparky.getOutputCurrent());

    // inputs.leftVolts = Volts.of(leftSparky.getAppliedOutput() * leftSparky.getBusVoltage());
    // inputs.rightVolts = Volts.of(rightSparky.getAppliedOutput() * rightSparky.getBusVoltage());

    // inputs.velocity = MetersPerSecond.of(encoder.getVelocity());
    // Logger.recordOutput("Elevator/RawRotations", leftSparky.getEncoder().getPosition());
  }

  @Override
  public void setVolts(double volts) {
    // leftSparky.setVoltage(volts);
  }

  @Override
  public void resetEncoder() {
    // encoder.setPosition(0.0);
  }
}
