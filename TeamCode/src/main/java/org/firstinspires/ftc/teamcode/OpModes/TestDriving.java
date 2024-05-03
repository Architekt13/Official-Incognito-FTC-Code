package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveCodeSubsystem;

@TeleOp
public class TestDriving extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        DriveCodeSubsystem driveCode = new DriveCodeSubsystem();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            driveCode.Driving();
        }
    }
}