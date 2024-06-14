package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class TestArm extends LinearOpMode{

    Servo leftArm;
    Servo rightArm;
    Servo secondary;
    public static double leftServoInit = 0;
    public static double intakePickPos = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        leftArm = hardwareMap.get(Servo.class, "left");
        rightArm = hardwareMap.get(Servo.class, "right");
        secondary = hardwareMap.get(Servo.class, "secondary");
        rightArm.setPosition(leftServoInit);
        leftArm.setPosition(1-leftServoInit);
        secondary.setPosition(1);


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            if(gamepad1.b){
                rightArm.setPosition(intakePickPos);
                leftArm.setPosition(1-intakePickPos);
                secondary.setPosition(0);
            }
            if(gamepad1.x){
                rightArm.setPosition(leftServoInit);
                leftArm.setPosition(1-leftServoInit);
                secondary.setPosition(1);
            }
            if(gamepad1.dpad_left){
                secondary.setPosition(1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  );
            }
            if(gamepad1.dpad_right){
                secondary.setPosition(0);
            }
        }
    }
}