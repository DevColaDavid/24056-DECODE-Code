package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

public class Intake extends SubsystemBase {
    Telemetry telemetry;
    private final MotorEx motor;
    private PIDController veloController;
    private SimpleMotorFeedforward feedforward;

    private double targetRpm = 0;
    private double integral = 0.0, prevErr = 0.0;
    public Intake(final HardwareMap hMap, final String motorName, Telemetry telemetry){
        veloController = new PIDController(RoboConstants.INTAKE_kP, RoboConstants.INTAKE_kI, RoboConstants.INTAKE_kD);
        feedforward = new SimpleMotorFeedforward(RoboConstants.INTAKE_kS, RoboConstants.INTAKE_kV, RoboConstants.INTAKE_kA);

        this.telemetry = telemetry;
        motor = new MotorEx(hMap,motorName, Motor.GoBILDA.RPM_1150);
        motor.setInverted(true);
        motor.setVeloCoefficients(RoboConstants.INTAKE_kP, RoboConstants.INTAKE_kI, RoboConstants.INTAKE_kD);
        motor.setFeedforwardCoefficients(RoboConstants.INTAKE_kS, RoboConstants.INTAKE_kV, RoboConstants.INTAKE_kA);

//        motor.setDirection(DcMotorEx.Direction.REVERSE);
//        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RobotLog.i("[Intake] Initialized.");
    }
    public void setTargetRpm(double rpm) { this.targetRpm = rpm; }
    public double getTargetRpm() { return targetRpm; }

    private double ticksPerSec(MotorEx m) {
        // Use velocity if available; otherwise differentiate position (SDK velocity is usually fine)
        return m.getVelocity();
    }

    public double getRpm() {
        double tps = ticksPerSec(motor);
        return (tps / RoboConstants.INTAKE_TPR) * 60.0;
    }

    double dt = 1.0 / RoboConstants.LOOP_HZ; // FTC OpMode loop ≈ 50Hz
    @Override
    public void periodic() {
        double rpm = getRpm();
//        double err = targetRpm - rpm;
//        integral += err * dt;
//        double deriv = (err - prevErr) / dt;
//        prevErr = err;
//
//        // Simple FF: convert target rpm -> ticks/s -> power scale
//        double ff = RoboConstants.INTAKE_kF * (targetRpm / 60.0) * RoboConstants.INTAKE_TPR;
//
//        double pwr = RoboConstants.INTAKE_kP * err + RoboConstants.INTAKE_kI * integral
//                + RoboConstants.INTAKE_kD * deriv + ff;
//
//        pwr = Math.max(0.0, Math.min(1.0, pwr)); // no reverse

        motor.set(targetRpm);

        telemetry.addLine("====== Intake ======");
        telemetry.addData("Intake/TargetRPM", "%.0f", targetRpm);
        telemetry.addData("Intake/RPM", "%.0f", rpm);
//        telemetry.addData("Intake/Power", "%.2f", pwr);
    }
}
