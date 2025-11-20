package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoboConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Configurable
@TeleOp(name="Decode Tele", group="Competition")
public class DecodeTeleOp extends OpMode {
    // ===== Drive =====
    private Follower follower;

    // ===== Limelight =====
//    private LimelightClient limelightClient;

    // ==== Other Subsystems ====
    private Turret turret;
    private FlyWheel flywheel;
    private Indexer indexer;
    private Intake intake;

//    private DcMotor lf,lb,rf,rb;

    // ===== Utils =====
    private GamepadEx driverOp;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
//        lf = hardwareMap.get(DcMotor.class, "leftFront");
//        lb = hardwareMap.get(DcMotor.class, "leftBack");
//        rf = hardwareMap.get(DcMotor.class, "rightFront");
//        rb = hardwareMap.get(DcMotor.class, "rightBack");

        // pedropath
        follower = Constants.createFollower(hardwareMap, telemetry);
        follower.update();

        // other subsystems
        turret = new Turret(hardwareMap, RoboConstants.TURRET_MOTOR_NAME, telemetry);
        flywheel = new FlyWheel(hardwareMap, RoboConstants.FLYWHEEL_A_MOTOR_NAME, RoboConstants.FLYWHEEL_B_MOTOR_NAME, telemetry);
        indexer = new Indexer(hardwareMap, RoboConstants.INDEXER_LEFT_SERVO_NAME, RoboConstants.INDEXER_RIGHT_SERVO_NAME, telemetry);

        intake = new Intake(hardwareMap, RoboConstants.INTAKE_MOTOR_NAME, telemetry);

        // util
        driverOp = new GamepadEx(gamepad1);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        loopTimer.reset();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        /*
         * Drive Stuff
         */
        follower.update();;
        follower.setTeleOpDrive(
                driverOp.getLeftY()*0.5,
                -driverOp.getLeftX()*0.5,
                -driverOp.getRightX()*0.5,
                true // Robot Centric
        );

        // subsystem
        if (driverOp.getButton(GamepadKeys.Button.Y)){
            flywheel.setTargetRpm(1000);
        }
        else if (driverOp.getButton(GamepadKeys.Button.X)){
            flywheel.setTargetRpm(0);
        }

        if (driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.3){
            intake.setTargetRpm(315);
        }
        else if (driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            intake.setTargetRpm(-300);
        }
        else{
            intake.setTargetRpm(0);
        }

        if(driverOp.getButton(GamepadKeys.Button.DPAD_LEFT)){
            turret.setMotorPwr(-0.8);
        }
        else if(driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            turret.setMotorPwr(0.8);
        }
        else {
            turret.setMotorPwr(0);
        }

        if(driverOp.getButton(GamepadKeys.Button.A)){
            indexer.hold();
        } else if (driverOp.getButton(GamepadKeys.Button.B)) {
            indexer.release();
        }

        turret.periodic();
        flywheel.periodic();
        indexer.periodic();
        intake.periodic();

        telemetry.addData("Pose", follower.getPose().toString());


//        if (driverOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
//            lf.setPower(0.25);
//            lb.setPower(0.25);
//            rf.setPower(0.25);
//            rb.setPower(0.25);
//        }
    }
}
