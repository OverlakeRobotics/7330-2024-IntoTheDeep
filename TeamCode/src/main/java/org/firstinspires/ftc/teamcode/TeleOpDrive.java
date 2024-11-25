package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Intake;

@TeleOp(name = "TeleOp")
public class TeleOpDrive extends OpMode {

    private boolean toggleA = false;
    private boolean toggleB = false;


    private boolean armMoving = false;
    private boolean extendMoving = false;
    private boolean intakeMoving = false;

    Intake intake;
    Drive drive;


    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        drive = new Drive(hardwareMap);

        intake.extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    @Override
    public void loop() {



        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        float theta = gamepad1.right_stick_x;

        drive.drive(x, y, theta);

        if (gamepad1.left_bumper) {
            if (!(intake.arm1.getPower() > 0.5)) {
                intake.arm1.setPower(0.3);
                intake.arm2.setPower(0.3);
                armMoving = true;
            }
        } else if (gamepad1.right_bumper) {
            if (!(intake.arm1.getPower() < -0.5)) {
                intake.arm1.setPower(-1);
                intake.arm2.setPower(-1);
                armMoving = true;
            }
        } else {
            intake.arm1.setPower(0);
            intake.arm2.setPower(0);
            armMoving = false;
        }

        if (gamepad1.right_trigger > 0.1) {
            if (!intakeMoving) {
                intake.intake();
                intakeMoving = true;
            }
        } else if (gamepad1.left_trigger > 0.1) {
            if (!intakeMoving) {
                intake.outtake();
                intakeMoving = true;
            }
        } else {
            intake.intakeStop();
            intakeMoving = false;
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
            intake.resetArmPos();
            intake.resetExtendPos();
        }
        if (gamepad1.x && gamepad1.y) {
            intake.extend.setPower(-1);
            intake.arm1.setPower(1);
            intake.arm2.setPower(1);
            //TODO: Hang Routine
        } else {
            intake.armStop();
            intake.extendStop();
        }


        if (gamepad1.dpad_up) {
//            if (intake.extend.getCurrentPosition() > 2500) {
//                intake.extend.setPower(0);
//            }
            //ELSE
            if (!(intake.extend.getPower() > 0.5)) {
                intake.extend.setPower(1);
            }

        }
        else if (gamepad1.dpad_down) {

            if (!(intake.arm1.getPower() < -0.5)) {
                intake.extend.setPower(-1);
                armMoving = true;
            }

        } else {
            intake.extend.setPower(0);
            extendMoving = false;
        }

        if (gamepad1.dpad_left) {
            if (intake.arm1.getCurrentPosition() < 775){
                if (!(intake.arm1.getPower() > 0.5)) {
                    intake.arm1.setPower(-0.85);
                    intake.arm2.setPower(-0.85);
                    armMoving = true;
                }
            }
            else {
                intake.arm1.setPower(0);
                intake.arm2.setPower(0);
            }

        }


        telemetryData();

    }

    private void telemetryData() {
        telemetry.addData("Arm Motor value:", intake.getArmPos());
        telemetry.addData("Extend Motor value:", intake.getExtendPos());

        telemetry.addData("Drive Speed:", drive.isSlowMode() ? "SLOW" : "FAST");
        telemetry.addData("Arm Speed:", intake.isSlowMode() ? "SLOW" : "FAST");
        telemetry.update();
    }
}
