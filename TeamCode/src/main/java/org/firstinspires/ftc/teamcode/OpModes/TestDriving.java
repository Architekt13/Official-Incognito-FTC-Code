package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Functions.SCurve;
import org.firstinspires.ftc.teamcode.Subsystems.DriveCodeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeTest;

@TeleOp
public class TestDriving extends LinearOpMode {
    MotorEx frontLeft;
    MotorEx frontRight;
    MotorEx backLeft;
    MotorEx backRight;
    IMU imu;
    double[] currentMotorDrives = {0, 0, 0, 0};
    public static double maxAccel = 0.05;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public enum RobotState {
        PIXEL_INTAKE,
        PIXEL_READY,
        PIXEL_READY_1,
        PIXEL_READY_2,
        PIXEL_READY_3,
        READY_TO_DEPOSIT,
        READY_TO_DEPOSIT2,
        PIXEL_DEPOSITING,
        PIXEL_READY_4,
        PIXEL_DEPOSITED,
        PIXEL_RETURN,
    }

    ;

    IntakeTest.RobotState robotState = IntakeTest.RobotState.PIXEL_INTAKE;
    MotorEx intakeMotor;
    Servo pitchIntake;
    public static double[] pitchIntakeVal = {0, 0.15, 0.3, 0.45, 0.6, 0.85};
    ElapsedTime outtakeTimer = new ElapsedTime();
    double intakeSpeedMod = 1;
    public static double[] delays = {3, 3, 3, 3, 3, 3, 3};
    public static double[] servoPos = {0.1, 1, 0.05, 0.4, 1, 1, 0.5, 0, 0, 1, 0.5, 0.8};
    public static double[] servoResetPos = {0.4, 1};
    Servo left;
    Servo right;
    Servo secondary;
    Servo claw;


    @Override
    public void runOpMode() throws InterruptedException {
        backRight = new MotorEx(hardwareMap, "backRight");
        backLeft = new MotorEx(hardwareMap, "backLeft");
        frontRight = new MotorEx(hardwareMap, "frontRight");
        frontLeft = new MotorEx(hardwareMap, "frontLeft");
        intakeMotor = new MotorEx(hardwareMap, "intake");
        frontRight.setInverted(true);
        backRight.setInverted(true);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            intakeMotor.set(gamepad1.right_trigger - gamepad1.left_trigger);
            if (gamepad1.start) {
                imu.resetYaw();
            }
            double leftX = gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y;
            double rightX = gamepad1.right_stick_x;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = leftX * Math.cos(-botHeading) - leftY * Math.sin(-botHeading);
            double rotY = leftX * Math.sin(-botHeading) + leftY * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);
            double frontLeftPower = (rotY + rotX + rightX) / denominator;
            double backLeftPower = (rotY - rotX + rightX) / denominator;
            double frontRightPower = (rotY - rotX - rightX) / denominator;
            double backRightPower = (rotY + rotX - rightX) / denominator;

            frontLeft.set(SCurve.Stabilize(frontLeftPower, currentMotorDrives[0], maxAccel));
            frontRight.set(SCurve.Stabilize(frontRightPower, currentMotorDrives[1], maxAccel));
            backLeft.set(SCurve.Stabilize(backLeftPower, currentMotorDrives[2], maxAccel));
            backRight.set(SCurve.Stabilize(backRightPower, currentMotorDrives[3], maxAccel));
            currentMotorDrives[0] = frontLeft.get();
            currentMotorDrives[1] = frontRight.get();
            currentMotorDrives[2] = backLeft.get();
            currentMotorDrives[3] = backRight.get();
            dashboardTelemetry.addData("Heading", botHeading);
            dashboardTelemetry.addData("FL", frontLeft.get());
            dashboardTelemetry.addData("FR", frontRight.get());
            dashboardTelemetry.addData("BR", backRight.get());
            dashboardTelemetry.addData("BL", backLeft.get());
            dashboardTelemetry.update();

        }

    }
}