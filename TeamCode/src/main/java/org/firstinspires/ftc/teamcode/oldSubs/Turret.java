package org.firstinspires.ftc.teamcode.oldSubs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RoboConstants;

public class Turret extends SubsystemBase{
    private final MotorEx turretMotor;
    private final ElapsedTime timer = new ElapsedTime();

    private double targetDeg = 0.0;
    private double integral = 0.0;
    private double prevErr = 0.0;

    private double motorPwr = 0.0;

    public Turret() {
        turretMotor = new MotorEx(hardwareMap,"turret", Motor.GoBILDA.RPM_1150);

        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.setInverted(false);
        RobotLog.i("[Turret] Initialized.");
        timer.reset();
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

    public void setMotorPower(double pwr){
        this.motorPwr= pwr;
        this.turretMotor.set(motorPwr);
    }

    /** Call each loop */
    public void periodic(double dt) {
        telemetry.addData("Turret/AngleDeg", "%.1f", getAngleDeg());
        telemetry.addData("Turret/TargetDeg", "%.1f", targetDeg);
        turretMotor.set(motorPwr);
    }
}