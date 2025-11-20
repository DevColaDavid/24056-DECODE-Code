package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public final class RoboConstants {
    // ===== Imu =====
    public static final RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    public static final RevHubOrientationOnRobot imuOrientationOnRobot = new
            RevHubOrientationOnRobot(logoDirection, usbDirection);

    // ===== Camera =====
    public static final String LIMELIGHT_NAME = "limelight";
    public static Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    // ====== Turret ======
    public static final String TURRET_MOTOR_NAME = "turret";
    public static final double TURRET_GEAR_RATIO = 0.2; // 20T:100T
    public static final int TURRET_TPR = ((((1+(46/17))) * (1+(46/17))) * 28); //goBILDA 435RPM
    public static final double TURRET_MAX_RPM = 87; // approx, for feed-forward
    // Soft limits (wrap logic allows 360°, but we use these to cap motion per update)
    public static final double TURRET_MAX_SPEED_DEG_PER_S = 522;

    // Turret PID (angle error to power) — START CONSERVATIVE, TUNE ON ROBOT
    public static final double TURRET_kP = 0.0001;
    public static final double TURRET_kI = 0.0;
    public static final double TURRET_kD = 0.02;
    public static final double TURRET_kF = 0.0;  // optional feedforward

    // Homing/search speed (sign decides direction). Keep gentle.
    public static final double TURRET_HOME_SEARCH_PWR = -0.12;
    public static final double TURRET_HOME_SETTLE_S  = 0.30;

    // ====== Flywheel ======
    public static final String FLYWHEEL_A_MOTOR_NAME = "flyWheelA";
    public static final String FLYWHEEL_B_MOTOR_NAME = "flyWheelB";
    public static final double FLYWHEEL_kP = 0.015;
    public static final double FLYWHEEL_kI = 0.0;
    public static final double FLYWHEEL_kD = 0.0;
    public static final double FLYWHEEL_kF = 0.0; // simple velocity feedforward (ticks/s -> power)
    // Rev through-bore enc or built-in? adjust TPR to your actual
    public static final int FLYWHEEL_TPR = 	-((1+(46/11)) * 28); //goBILDA 5203 1150RPM
    public static final double FLYWHEEL_TARGET_RPM_DEFAULT = 0;

    // ===== Intake =====
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final double INTAKE_kP = 0.001;
    public static final double INTAKE_kI = 0.0;
    public static final double INTAKE_kD = 0.0;
    public static final double INTAKE_kS = 0.0;
    public static final double INTAKE_kV = 0.0;
    public static final double INTAKE_kA = 0.0;
    public static final double INTAKE_kF = 0.0; // simple velocity feedforward (ticks/s -> power)
    // Rev through-bore enc or built-in? adjust TPR to your actual
    public static final int INTAKE_TPR = ((1+(46/11))* 28); //goBILDA 5203 1150RPM

    // ===== Indexer =====
    public static final String INDEXER_LEFT_SERVO_NAME = "indexerLeft";
    public static final String INDEXER_RIGHT_SERVO_NAME = "indexerRight";

    // ====== Utilities ======
    public static final double LOOP_HZ = 50.0;
    public static final double dt(ElapsedTime t) { return t.seconds(); }
}