package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoboConstants;

public class Turret extends SubsystemBase {
    private Telemetry telemetry;
    private double motorPwr = 0.0;
    private final DcMotor turretMotor;
    private double targetDeg = 0.0;
    private double integral = 0.0;
    private double prevErr = 0.0;

    public Turret(final HardwareMap hMap, final String turretMotorName, Telemetry telemetry){
        this.telemetry = telemetry;
        turretMotor = hMap.get(DcMotor.class, turretMotorName);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RobotLog.i("[Turret] Initialized.");
    }

    /** Convert motor ticks to output degrees (update for your gear ratio) */
    public double ticksToDeg(int ticks) {
        double revs = ticks / (double) RoboConstants.TURRET_TPR;
        double outRevs = revs / RoboConstants.TURRET_GEAR_RATIO;
        return outRevs * 360.0;
    }

    /** Convert degrees to motor ticks */
    public int degToTicks(double deg) {
        double outRevs = deg / 360.0;
        double motorRevs = outRevs * RoboConstants.TURRET_GEAR_RATIO;
        return (int) Math.round(motorRevs * RoboConstants.TURRET_TPR);
    }

    public double getAngleDeg() {
        return ticksToDeg(turretMotor.getCurrentPosition());
    }

    public void setTargetDeg(double deg) {
        targetDeg = deg;
    }

    public void setMotorPwr(double pwr){
        motorPwr = pwr;
    }

    double dt = 1.0 / RoboConstants.LOOP_HZ; // FTC OpMode loop ≈ 50Hz
    @Override
    public void periodic() {
//        // PID angle hold / tracking
//        double currentDeg = getAngleDeg();
//        double err = AngleUtil.shortestDiffDeg(targetDeg, currentDeg);
//        integral += err * dt;
//        double deriv = (err - prevErr) / dt;
//        prevErr = err;
//
//        double cmd = Constants.TURRET_kP * err
//                + Constants.TURRET_kI * integral
//                + Constants.TURRET_kD * deriv
//                + Constants.TURRET_kF * 0.0;
//
//        // Constrain final velocity (soft cap)
//        cmd = AngleUtil.clamp(cmd, -0.6, 0.6);
//        turretMotor.setPower(cmd);

        turretMotor.setPower(motorPwr);

        telemetry.addLine("======Turret======");
        telemetry.addData("Turret/AngleDeg", "%.1f", getAngleDeg());
        telemetry.addData("Turret/TargetDeg", "%.1f", targetDeg);
    }
}
