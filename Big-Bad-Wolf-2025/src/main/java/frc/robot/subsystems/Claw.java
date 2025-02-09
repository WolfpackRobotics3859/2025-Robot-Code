// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.Hardware;
import frc.robot.subsystems.claw.ClawRequest;

public class Claw extends SubsystemBase {

  private final TalonFX m_ArmMotor;
  private final TalonFX m_RollerMotor;
  
  private SysIdRoutine sysIdRoutine;


  /** Creates a new Claw. */
  public Claw() {
    m_RollerMotor = new TalonFX(Hardware.CLAW_ROLLER_MOTOR_ID);
    m_ArmMotor = new TalonFX(Hardware.CLAW_ARM_MOTOR_ID);

    m_ArmMotor.getConfigurator().apply(ClawConstants.ALGAE_INTAKE_WRIST_GAINS);

  }

  public Command armGoToPosition(double inputPosition) {
     return this.run(() -> this.applyRequest(new ClawRequest().position(inputPosition), m_ArmMotor));
  }

  public Command spinIntakeWheels(double inputPercentOut) {
    return this.run(() -> this.applyRequest(new ClawRequest().percentOut(inputPercentOut), m_RollerMotor));
  }

  public Command placeCoral(){
    return this.startEnd(
      () -> this.armGoToPosition(ClawConstants.CLAW_ARM_PLACE_POSTION),
      () -> this.spinIntakeWheels(ClawConstants.CLAW_ROLLER_EXPEL_CORAL)
    );
  }

  public double getArmPosition() {
    return m_ArmMotor.getPosition().getValueAsDouble();
  }

  public double getRollerDirection() {
    return m_RollerMotor.getVelocity().getValueAsDouble();
  }

  private void applyRequest(ClawRequest clawRequest, TalonFX requestedMotor) {

    ControlRequest controlRequest = null;

    switch (clawRequest.getClawRequestType()) {
      case NOOP:
        controlRequest = new DutyCycleOut(0);
      case BRAKE:
        controlRequest = new StaticBrake();
        break;
      case POSITION:
        controlRequest = new MotionMagicVoltage(clawRequest.getValue());
        break;
      case PERCENT:
        controlRequest = new DutyCycleOut(clawRequest.getValue());
        break;
      case VOLTAGE:
        controlRequest = new VoltageOut(clawRequest.getValue());
        break;
    }
    if(controlRequest != null)
      requestedMotor.setControl(controlRequest);
    else
      System.out.println("error controlRequest: null");
  }

  public SysIdRoutine builSysIdRoutine() {
    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(4),
        null,
        (state) -> SignalLogger.writeString("state", state.name())
      ),
      new SysIdRoutine.Mechanism(
        (volts) -> m_ArmMotor.setControl(new VoltageOut(volts.in(Volts))),
        null,
        this
      )
    );

    return sysIdRoutine;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
