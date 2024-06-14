package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveCodeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeTest;

@TeleOp
public class TestIntake extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeTest intake = new IntakeTest();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            intake.Intaking();
        }
    }
}