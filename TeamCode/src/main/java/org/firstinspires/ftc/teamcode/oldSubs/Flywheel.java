package org.firstinspires.ftc.teamcode.oldSubs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RoboConstants;

public class Flywheel {
    private final DcMotorEx a, b;
    private double targetRpm = RoboConstants.FLYWHEEL_TARGET_RPM_DEFAULT;
    private double integral = 0.0, prevErr = 0.0;

    public Flywheel() {
        a = hardwareMap.get(DcMotorEx.class, RoboConstants.FLYWHEEL_A_MOTOR_NAME);
        b = hardwareMap.get(DcMotorEx.class, RoboConstants.FLYWHEEL_B_MOTOR_NAME);
        a.setDirection(DcMotorEx.Direction.FORWARD);
        b.setDirection(DcMotorEx.Direction.REVERSE);
        a.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        b.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        a.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        b.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        a.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        b.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        RobotLog.i("[Flywheel] Initialized.");
    }
    public void setTargetRpm(double rpm) { this.targetRpm = rpm; }
    public double getTargetRpm() { return targetRpm; }

    private double ticksPerSec(DcMotorEx m) {
        // Use velocity if available; otherwise differentiate position (SDK velocity is usually fine)
        return m.getVelocity();
    }

    public double getRpm() {
        double tps = 0.5 * (ticksPerSec(a) + ticksPerSec(b));
        return (tps / RoboConstants.FLYWHEEL_TPR) * 60.0;
    }

    public void periodic(double dt) {
        double rpm = getRpm();
        double err = targetRpm - rpm;
        integral += err * dt;
        double deriv = (err - prevErr) / dt;
        prevErr = err;

        // Simple FF: convert target rpm -> ticks/s -> power scale
        double ff = RoboConstants.FLYWHEEL_kF * (targetRpm / 60.0) * RoboConstants.FLYWHEEL_TPR;

        double pwr = RoboConstants.FLYWHEEL_kP * err + RoboConstants.FLYWHEEL_kI * integral
                + RoboConstants.FLYWHEEL_kD * deriv + ff;

        pwr = Math.max(0.0, Math.min(1.0, pwr)); // no reverse

        a.setPower(pwr);
        b.setPower(pwr);

        telemetry.addData("Flywheel/TargetRPM", "%.0f", targetRpm);
        telemetry.addData("Flywheel/RPM", "%.0f", rpm);
        telemetry.addData("Flywheel/Power", "%.2f", pwr);
    }
}