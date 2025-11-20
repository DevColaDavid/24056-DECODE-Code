package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Auto Code")
public class autoOp extends OpMode {
    // DriveMotor
    private Motor frontLeft, frontRight, backLeft, backRight;
    // FlyWheel
    private Motor flyWheel1, flyWheel2;
    // Intake
    private Motor intake;

    private Motor turretRot;
    private MecanumDrive drive;
    private GamepadEx driverOp, turretOp;
    private ServoEx indexLeft, indexRight;


    @Override
    public void init() {
        frontLeft = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435);
        frontRight = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435);
        backLeft = new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435);
        backRight = new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_435);
        frontLeft.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(true);
        backRight.setInverted(true);

        flyWheel1 = new MotorEx(hardwareMap,"flyWheelUp", Motor.GoBILDA.RPM_1150);
        flyWheel2 = new MotorEx(hardwareMap, "flyWheelDown", Motor.GoBILDA.RPM_1150);
        flyWheel1.setInverted(true);
        flyWheel1.setRunMode(Motor.RunMode.VelocityControl);
        flyWheel2.setRunMode(Motor.RunMode.VelocityControl);

        intake = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);
        intake.setRunMode(Motor.RunMode.VelocityControl);
        intake.setInverted(true);

        turretRot = new Motor(hardwareMap, "turretRot", Motor.GoBILDA.RPM_1150);
        turretRot.setInverted(true);
        turretRot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretRot.setRunMode(Motor.RunMode.VelocityControl);

        indexLeft = new SimpleServo(hardwareMap, "indexLeft",0,300);
        indexRight = new SimpleServo(hardwareMap, "indexRight",0,300);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        driverOp = new GamepadEx(gamepad1);
        turretOp = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        frontLeft.set(0.1);
        frontRight.set(0.1);
        backLeft.set(0.1);
        backRight.set(0.1);
    }
}
