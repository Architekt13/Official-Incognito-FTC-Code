package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Functions.SCurve;

@Config
public class ExtendoSlides extends SubsystemBase {
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;
    GamepadEx driveGamePad = new GamepadEx(gamepad1);
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    int targetPos = 0;
    public static int step =5;
    public ExtendoSlides(){
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void Extending(){
        if(gamepad1.dpad_up){
            targetPos+=step;
        } else if (gamepad1.dpad_down) {
            targetPos-=step;
        }
        leftSlide.setTargetPosition(targetPos);
        rightSlide.setTargetPosition(targetPos);
        dashboardTelemetry.addData("Left Slide Pos", leftSlide.getCurrentPosition());
        dashboardTelemetry.addData("Right Slide Pos", rightSlide.getCurrentPosition());
        dashboardTelemetry.update();
    }
}
