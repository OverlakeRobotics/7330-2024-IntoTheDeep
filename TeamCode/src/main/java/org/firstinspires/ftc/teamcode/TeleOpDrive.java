package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Intake;

@TeleOp(name = "TeleOp")
public class TeleOpDrive extends OpMode {

    private boolean toggleA = false;
    private boolean toggleB = false;


    Intake intake;
    Drive drive;


    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        drive = new Drive(hardwareMap);

    }



    @Override
    public void loop() {

        float x = gamepad1.right_stick_x;
        float y = gamepad1.right_stick_y;
        float theta = gamepad1.left_stick_x;

        drive.drive(x, y, theta);

        if (gamepad1.right_bumper) {
            intake.driveArmUp();
        }
        if (gamepad1.left_bumper) {
            intake.driveArmDown();
        }

        if (gamepad1.right_trigger > 0.1) {
            intake.intake();
        }
        if (gamepad1.left_trigger > 0.1) {
            intake.outtake();
        }

        if (gamepad1.a && !toggleA) {
            drive.setSlowMode(!drive.isSlowMode());
            toggleA = true;
        } else if (!gamepad1.a) {
            toggleA = false;
        }

        if (gamepad1.b && !toggleB) {
            intake.setSlowMode(!drive.isSlowMode());
            toggleB = true;
        }else if (!gamepad1.a) {
            toggleB = false;
        }

        if (gamepad1.x) {
            //NOTHING
        }
        if (gamepad1.y) {
            //TODO: Hang Routine
        }


        if (gamepad1.dpad_up) {
            intake.driveExtendUp();
        }
        if (gamepad1.dpad_down) {
            intake.driveExtendDown();
        }



    }
}
