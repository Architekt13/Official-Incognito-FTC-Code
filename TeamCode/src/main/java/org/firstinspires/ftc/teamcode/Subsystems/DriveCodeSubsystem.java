package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Functions.SCurve;

@Config
public class DriveCodeSubsystem extends SubsystemBase {
    MotorEx frontLeft;
    MotorEx frontRight;
    MotorEx backLeft;
    MotorEx backRight;
    IMU imu;
    GamepadEx driveGamePad = new GamepadEx(gamepad1);
    double[] currentMotorDrives = {0,0,0,0};
    public static double maxAccel = 0.05;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public DriveCodeSubsystem(){
        backRight = new MotorEx (hardwareMap, "backRight");
        backLeft = new MotorEx (hardwareMap, "backLeft");
        frontRight = new MotorEx (hardwareMap, "frontRight");
        frontLeft = new MotorEx (hardwareMap, "frontLeft");
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
    }
    public void Driving(){
        if(driveGamePad.getButton(GamepadKeys.Button.START)){
            imu.resetYaw();
        }
        double leftX = driveGamePad.getLeftX();
        double leftY = driveGamePad.getLeftY();
        double rightX = driveGamePad.getRightX();
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

        frontLeft.set(SCurve.Stabilize(frontLeftPower, currentMotorDrives[1], maxAccel));
        frontRight.set(SCurve.Stabilize(frontRightPower, currentMotorDrives[2], maxAccel));
        backLeft.set(SCurve.Stabilize(backLeftPower, currentMotorDrives[3], maxAccel));
        backRight.set(SCurve.Stabilize(backRightPower, currentMotorDrives[4], maxAccel));
        currentMotorDrives[1] = frontLeftPower;
        currentMotorDrives[2] = frontRightPower;
        currentMotorDrives[3] = backLeftPower;
        currentMotorDrives[4] = backRightPower;
        dashboardTelemetry.addData("Heading", botHeading);
        dashboardTelemetry.addData("FL",frontLeft.get());
        dashboardTelemetry.addData("FR",frontRight.get());
        dashboardTelemetry.addData("BR",backRight.get());
        dashboardTelemetry.addData("BL",backLeft.get());
        dashboardTelemetry.update();
    }
}
