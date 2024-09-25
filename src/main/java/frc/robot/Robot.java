// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Robot extends TimedRobot {
    private FeedforwardSim neoSim;
    private FeedforwardSim krakenSim;
    private CANSparkMax neo;
    private RelativeEncoder encoder;
    private TalonFX kraken;
    private VoltageOut krakenVolts;
    private double voltage;
    private Subsystem hi = new SubsystemBase() {};
    public Robot() {
        neoSim = FeedforwardSim.createFlywheel(0.11361, 0.12161, 0.013461, new State());
        krakenSim = FeedforwardSim.createFlywheel(0.0013844, 0.11547, 0.005188, new State());
        neo = new CANSparkMax(3, MotorType.kBrushless);
        encoder = neo.getEncoder();
        kraken = new TalonFX(4);
        krakenVolts = new VoltageOut(0);
        encoder.setPosition(0);
        kraken.setPosition(0);

        SmartDashboard.putData("0 Volts", setVoltage(0));
        SmartDashboard.putData("2.5 Volts", setVoltage(2.5));
        SmartDashboard.putData("5 Volts", setVoltage(5));
        SmartDashboard.putData("7.5 Volts", setVoltage(7.5));
        SmartDashboard.putData("10 Volts", setVoltage(10));
        SmartDashboard.putData("-2.5 Volts", setVoltage(-2.5));
        SmartDashboard.putData("-5 Volts", setVoltage(-5));
        SmartDashboard.putData("-7.5 Volts", setVoltage(-7.5));
        SmartDashboard.putData("-10 Volts", setVoltage(-10));
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Neo Sim Position", neoSim.getPosition());
        SmartDashboard.putNumber("Neo Sim Velocity", neoSim.getVelocity());
        SmartDashboard.putNumber("Neo Real Position", encoder.getPosition());
        SmartDashboard.putNumber("Neo Real Velocity", encoder.getVelocity() / 60);
        SmartDashboard.putNumber("Kraken Sim Position", krakenSim.getPosition());
        SmartDashboard.putNumber("Kraken Sim Velocity", krakenSim.getVelocity());
        SmartDashboard.putNumber("Kraken Real Position", kraken.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Kraken Real Velocity", kraken.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Voltage", voltage);
    }
    
    @Override
    public void teleopPeriodic() {
        neoSim.periodic();
        krakenSim.periodic();
    }

    private Command setVoltage(double voltage) {
        return Commands.runOnce(() -> {
            this.voltage = voltage;
            kraken.setControl(krakenVolts.withOutput(voltage));
            neo.setVoltage(voltage);
            neoSim.setVoltage(voltage);
            krakenSim.setVoltage(voltage);
        }, hi);
    }
}
