package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeTest;

@TeleOp
public class TestClaw extends LinearOpMode{

    Servo clawServo;
    @Override
    public void runOpMode() throws InterruptedException {
        clawServo = hardwareMap.get(Servo.class, "claw");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            if (gamepad1.left_bumper){
                clawServo.setPosition(1);
            } else if (gamepad1.right_bumper){
                clawServo.setPosition(0);
            }
        }
    }
}