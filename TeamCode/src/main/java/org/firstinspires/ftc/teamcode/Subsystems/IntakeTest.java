package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class IntakeTest extends LinearOpMode {
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
    };
    RobotState robotState = RobotState.PIXEL_INTAKE;
    DcMotorEx intakeMotor;
    Servo pitchIntake;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public static double[] pitchIntakeVal = {0,0.15,0.3,0.45,0.6,0.85};
    ElapsedTime outtakeTimer = new ElapsedTime();
    double intakeSpeedMod = 1;
    public static double[] delays = {3,3,3,3,3,3,3};
    public static double [] servoPos = {0.1,1,0.05,0.4,1,1,0.5,0,0,1,0.5,0.8};
    public static double [] servoResetPos = {0.5, 1};
    Servo left;
    Servo right;
    Servo secondary;
    Servo claw;

    public void runOpMode() throws InterruptedException{
        outtakeTimer.reset();
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        secondary = hardwareMap.get(Servo.class, "secondary");
        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            Intaking();
        }

    }
    public void Intaking(){
        switch (robotState) {
            case PIXEL_INTAKE:
                //Intaking
                intakeMotor.setPower((gamepad1.right_trigger-gamepad1.left_trigger)/intakeSpeedMod);
                left.setPosition(1-servoPos[0]);
                right.setPosition(servoPos[0]);
                secondary.setPosition(servoPos[1]);
                claw.setPosition(1);
                if(gamepad1.a){
                    outtakeTimer.reset();
                    robotState = RobotState.PIXEL_READY_1;
                }
                break;
            case PIXEL_READY:
                if(outtakeTimer.seconds()>=delays[5]) {
                    secondary.setPosition(servoPos[9]);
                    outtakeTimer.reset();
                    robotState = RobotState.PIXEL_READY_3;
                }
              break;
            case PIXEL_READY_1:
                if(outtakeTimer.seconds()>=delays[6]) {
                    left.setPosition(1 - servoPos[2]);
                    right.setPosition(servoPos[2]);
                    outtakeTimer.reset();
                    robotState = RobotState.PIXEL_READY_2;
                }
                break;
            case PIXEL_READY_2:
                if(outtakeTimer.seconds()>=delays[0]){
                    claw.setPosition(0);
                    outtakeTimer.reset();
                    robotState = RobotState.PIXEL_READY;
                }
                break;
            case PIXEL_READY_3:
                if(outtakeTimer.seconds()>=delays[1]){
                    left.setPosition(1-servoPos[3]);
                    right.setPosition(servoPos[3]);
                    secondary.setPosition(servoPos[4]);
                    outtakeTimer.reset();
                    robotState = RobotState.READY_TO_DEPOSIT;
                }
                break;
            case READY_TO_DEPOSIT:
                if(gamepad1.y){
                    secondary.setPosition(servoPos[6]);
                    outtakeTimer.reset();
                    robotState = RobotState.READY_TO_DEPOSIT2;
                }
                break;
            case READY_TO_DEPOSIT2:
                if(outtakeTimer.seconds()>=delays[4]){
                    secondary.setPosition(servoPos[7]);
                    left.setPosition(1-servoPos[5]);
                    right.setPosition(servoPos[5]);
                    outtakeTimer.reset();
                    robotState = RobotState.PIXEL_DEPOSITING;
                }
                break;
            case PIXEL_DEPOSITING:
                if(outtakeTimer.seconds()>=delays[2]){
                    if (gamepad1.x){
                        while(gamepad1.x) {
                            claw.setPosition(1);
                        }
                        outtakeTimer.reset();
                        robotState = RobotState.PIXEL_DEPOSITED;
                    }
                }
                break;
            case PIXEL_DEPOSITED:
                if(outtakeTimer.seconds()>=delays[0]){
                    left.setPosition(1-servoResetPos[0]);
                    right.setPosition(servoResetPos[0]);
                    secondary.setPosition(servoResetPos[1]);
                    outtakeTimer.reset();
                    robotState = RobotState.PIXEL_RETURN;
                }
                break;
            case PIXEL_RETURN:
                if(outtakeTimer.seconds()>=delays[0]){
                    robotState = RobotState.PIXEL_INTAKE;
                }
                break;
        }
        dashboardTelemetry.addData("Intake Speed", intakeMotor.getPower());
        dashboardTelemetry.update();
    }
}
