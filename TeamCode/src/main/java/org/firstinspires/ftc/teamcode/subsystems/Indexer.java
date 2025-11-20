package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Indexer extends SubsystemBase {
    Telemetry telemetry;
    private final Servo indexerLeft, indexerRight;
    private double angleRange = 0.0; //Degree
    public Indexer(final HardwareMap hMap, final String IndexerLeftName, final String IndexerRightName, Telemetry telemetry){
        this.telemetry = telemetry;
        indexerLeft = hMap.get(Servo.class, IndexerLeftName);
        indexerRight = hMap.get(Servo.class, IndexerRightName);
        indexerRight.setDirection(Servo.Direction.FORWARD);
        indexerLeft.setDirection(Servo.Direction.REVERSE);
        indexerLeft.setPosition(0);
        indexerRight.setPosition(0);
        RobotLog.i("[Indexer] Initialized.");
    }

    public double getAngleDegrees(){
        return indexerRight.getPosition();
    }

    public void setServoZeroToOne(double AngleRange){
        angleRange = AngleRange;
    }

    public void hold(){
        setServoZeroToOne(0);
    }

    public void release(){
        setServoZeroToOne(0.8);
    }

    @Override
    public void periodic() {
        indexerLeft.setPosition(angleRange);
        indexerRight.setPosition(angleRange);

        telemetry.addLine("====== Indexer ======");
        telemetry.addData("targetAngle",angleRange);
        telemetry.addData("currentAngle",getAngleDegrees());
        telemetry.addData("indexerLeftAngle",indexerLeft.getPosition());
        telemetry.addData("indexerRightAngle",indexerRight.getPosition());
    }
}
