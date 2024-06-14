package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveCodeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ExtendoSlides;

@TeleOp
public class TestExtendo extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        ExtendoSlides Extendo = new ExtendoSlides();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            Extendo.Extending();
        }
    }
}