package org.firstinspires.ftc.teamcode.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public static final int ARM_GROUND_POS = 0;
    //public static final int ARM_INTAKE_POS = 250;
    public static final int ARM_LIFT_POS = 250;
    public static final int ARM_LOW_BASKET_POS = 3000;
    public static final int ARM_HIGH_BASKET_POS = 1825;

    public static final int ARM_CONTRACT_POS = 0;
    public static final int ARM_EXTEND_INTAKE_POS = 800;
    public static final int ARM_EXTEND_POS = 3475;

    public static final double ARM_POWER = 1d;
    public static final double ARM_EXTEND_POWER = 1d;
    public static final double ARM_CONTRACT_POWER = -1d;
    public static final double INTAKE_POWER = -1d;
    public static final double OUTTAKE_POWER = 0.7d;

    private static final double SLOW_FACTOR = 0.5;
    private static final int EXTEND_MAX = 3500;

    private boolean slowMode;


    private int armPosLeft;
    private int armPosRight;

    private DcMotor arm1;
    private DcMotor arm2;
    private DcMotor extend;
    private DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        this.arm1 = hardwareMap.get(DcMotor.class, "leftArm");
        this.arm2 = hardwareMap.get(DcMotor.class, "rightArm");
        this.extend = hardwareMap.get(DcMotor.class, "armExtend");
        this.intake = hardwareMap.get(DcMotor.class, "intake");


        initMotors();

    }

    public void initMotors() {
        arm1.setDirection(DcMotor.Direction.REVERSE);
        arm2.setDirection(DcMotor.Direction.FORWARD);
        extend.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        arm1.setTargetPosition(0);
//        arm2.setTargetPosition(1);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm1.setPower(0);
        arm2.setPower(0);
        extend.setPower(0);
        intake.setPower(0);
    }

    public boolean extendArm() {
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(extend, ARM_EXTEND_POS, ARM_EXTEND_POWER);
    }
    public boolean contractArm() {
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(extend, ARM_CONTRACT_POS, ARM_CONTRACT_POWER);
    }
    public boolean extendToPos(int pos, double power) {
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(extend, pos, power);
    }

    public boolean armToGround() {
        return armToPos(ARM_GROUND_POS, ARM_POWER);
    }
    public boolean liftArm() {
        return armToPos(ARM_LIFT_POS, ARM_POWER);
    }
    public boolean armToLowBasket() {
        return armToPos(ARM_LOW_BASKET_POS, ARM_POWER);
    }
    public boolean armToHighBasket() {
        return armToPos(ARM_HIGH_BASKET_POS, ARM_POWER);
    }
    public boolean armToPos(int pos, double power) {
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return driveToPos(arm1, arm2, pos, power);
    }

    public void intake() {
        intake.setPower(INTAKE_POWER);
    }
    public void outtake() {
        intake.setPower(OUTTAKE_POWER);
    }
    public void intakeStop() {
        intake.setPower(0);
    }

    public void driveArm(double power) {
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1.setPower(power);
    }

    public void driveArmUp() {
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1.setPower(ARM_POWER * (slowMode ? SLOW_FACTOR : 1));
        arm2.setPower(ARM_POWER * (slowMode ? SLOW_FACTOR : 1));

    }
    public void driveArmDown() {
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1.setPower(-ARM_POWER * (slowMode ? SLOW_FACTOR : 1));
        arm2.setPower(-ARM_POWER * (slowMode ? SLOW_FACTOR : 1));
    }
    public void armStop () {
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm1.setPower(0);
        arm2.setPower(0);
    }


    public void driveExtend(double power) {
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setPower(power * (slowMode ? SLOW_FACTOR : 1));
    }

    public void driveExtendUp() {
        if (getExtendPos() > EXTEND_MAX) {
            extendStop();
        } else {
            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extend.setPower(ARM_EXTEND_POWER * (slowMode ? SLOW_FACTOR : 1));
        }
    }
    public void driveExtendDown() {
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setPower(ARM_CONTRACT_POWER * (slowMode ? SLOW_FACTOR : 1));
    }
    public void extendStop() {
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setPower(0);
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    public void setSlowMode (boolean slow) {
        slowMode = slow;
    }


    public int getArmPos() {
        return arm1.getCurrentPosition();
    }
    public int getExtendPos() {
        return extend.getCurrentPosition();
    }
    public void resetArmPos() {
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetExtendPos() {
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public boolean driveToPos(DcMotor motor, int targetPos, double power) {
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            throw new IllegalArgumentException(String.format(
                    "Motor %s must be set to \"RUN_TO_POSITION\" mode", motor.getDeviceName())
            );
        }

        motor.setTargetPosition(targetPos);
        motor.setPower(power);

        int offset = Math.abs(motor.getCurrentPosition() - targetPos);
        return offset < 50;
    }

    public boolean driveToPos(DcMotor motor1, DcMotor motor2, int targetPos, double power) {
        if (motor1.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            throw new IllegalArgumentException(String.format(
                    "Motor %s must be set to \"RUN_TO_POSITION\" mode", motor1.getDeviceName())
            );
        }

        motor1.setTargetPosition(targetPos);
        motor1.setPower(power);
        motor2.setTargetPosition(targetPos);
        motor2.setPower(power);

        int offset = Math.abs(motor1.getCurrentPosition() - targetPos);
        return offset < 50;
    }

    /**************************************************************************
     ************************ AUTON ACTION CLASSES ****************************
     **************************************************************************/

    public Action armToFloorAuto() {
        return new ArmToFloorAuto();
    }
    public Action armToLiftAuto() {
        return new ArmToLiftAuto();
    }
    public Action armToHighBasketAuto() {
        return new ArmToHighBasketAuto();
    }
    public Action armToLowBasketAuto() {
        return new ArmToLowBasketAuto();
    }
    public Action contractAuto() {
        return new ContractAuto();
    }
    public Action extendToIntakeAuto() {
        return new ExtendToIntakeAuto();
    }
    public Action extendAuto() {
        return new ExtendAuto();
    }
    public Action intakeAuto () {
        return new IntakeAuto();
    }
    public Action outtakeAuto () {
        return new OuttakeAuto();
    }

    // Arm Classes
    public class ArmToFloorAuto implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm1.setPower(-ARM_POWER);
                initialized = true;
            }


            int pos = getArmPos();
            packet.put("armPos", pos);

            if (pos > 0) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }


    }
    public class ArmToLiftAuto implements Action {


        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                arm1.setTargetPosition(ARM_LIFT_POS);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (getArmPos() > ARM_LIFT_POS) {
                    arm1.setPower(-ARM_POWER);
                } else {
                    arm1.setPower(ARM_POWER);
                }
                initialized = true;
            }


            int pos = getArmPos();
            packet.put("armPos", pos);

            if (pos > ARM_LIFT_POS+50 || pos < ARM_LIFT_POS - 50) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }

    }
    public class ArmToLowBasketAuto implements Action {


        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm1.setPower(ARM_POWER);
                initialized = true;
            }


            int pos = getArmPos();
            packet.put("armPos", pos);

            if (pos < ARM_LOW_BASKET_POS) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }

    }
    public class ArmToHighBasketAuto implements Action {


        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!initialized) {
                //armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm1.setPower(ARM_POWER);
                //armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm1.setTargetPosition(ARM_HIGH_BASKET_POS);
                initialized = true;
            }


            int pos = getArmPos();
            packet.put("armPos", pos);

            if (pos < ARM_HIGH_BASKET_POS) {
                return true;
            } else {
                //armLeft.setPower(0);
                //armRight.setPower(0);
                return false;
            }
        }

    }


    // Extend classes
    public class ContractAuto implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                extend.setTargetPosition(ARM_CONTRACT_POS);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                extend.setPower(ARM_CONTRACT_POWER);
                initialized = true;
            }


            int pos = getExtendPos();
            packet.put("extendPos", pos);

            if (pos > ARM_CONTRACT_POS + 50) {
                return true;
            } else {
                extend.setPower(0);
                return false;
            }
        }


    }

    public class ExtendToIntakeAuto implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {

                extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (getExtendPos() > ARM_EXTEND_INTAKE_POS) {
                    arm1.setPower(ARM_CONTRACT_POWER);
                } else {
                    arm1.setPower(ARM_EXTEND_POWER);
                }
                initialized = true;
            }


            int pos = getExtendPos();
            packet.put("extendPos", pos);

            if (pos < ARM_EXTEND_INTAKE_POS+50 && pos > ARM_EXTEND_INTAKE_POS - 50) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }


    }

    public class ExtendAuto implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                extend.setTargetPosition(ARM_EXTEND_POS);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                extend.setPower(ARM_EXTEND_POWER);
                initialized = true;
            }


            int pos = getExtendPos();
            packet.put("extendPos", pos);

            if (pos < ARM_EXTEND_POS-50) {
                return true;
            } else {
                extend.setPower(0);
                return false;
            }
        }


    }

    // Intake Actions
    public class IntakeAuto implements Action {

        private boolean initialized = false;
        private int i = 0;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intake.setPower(INTAKE_POWER);
                initialized = true;
            }

            packet.put("intakeCounter", i);

            if (i < 10000) {
                i += 1;
                return true;
            } else {
                intake.setPower(0);
                return false;
            }
        }


    }
    public class OuttakeAuto implements Action {

        private boolean initialized = false;
        private int i = 0;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intake.setPower(OUTTAKE_POWER);
                initialized = true;
            }

            intake.setPower(1);

            packet.put("outtakeCounter", i);




            if (i < 10000) {
                i += 1;
                return true;
            } else {
                intake.setPower(0);
                return false;
            }
        }


    }

}
