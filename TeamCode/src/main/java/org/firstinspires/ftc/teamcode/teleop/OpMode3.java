package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

//test
@TeleOp(name = "OpMode3")

public class OpMode3 extends LinearOpMode {
    public ElapsedTime mRunTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();

    private int sleepMs1 = 0;
//
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // robot.liftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.liftHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // robot.liftHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x * -1; // -0.7
            double vertical = -gamepad1.left_stick_y * 1; // 0.7
            double turn = gamepad1.right_stick_x * 0.7;

            robot.setDrivePower(vertical + turn - horizontal, vertical - turn + horizontal, vertical + turn + horizontal, vertical - turn - horizontal);

            telemetry.addLine(String.format("FL: %d \nBL %d \nFR: %d \nBR: %d ",
                    robot.motorfl.getCurrentPosition(),
                    robot.motorbl.getCurrentPosition(),
                    robot.motorfr.getCurrentPosition(),
                    robot.motorbr.getCurrentPosition()
            ));
            // 0.7      1                    -0.7
            robot.setLiftPower((gamepad2.left_stick_y * 0.5), (gamepad2.left_stick_y * -0.5));
//make sure one of the directions is correct/reversed


            //spin

            //tilt bucket


            if (gamepad2.x) {
                robot.bucketTilt.setPosition(1);
            }

            if (gamepad2.b) {
                robot.bucketTilt.setPosition(0);
            }


            //lift arm start
            if (gamepad2.a) { //if button a pressed
                robot.intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.intakeServo.setPower(0.95);
            }
            else {
                robot.intakeServo.setPower(0);
            }

            if (gamepad2.y) {
                robot.intakeServo.setPower(-1);
            }
            else {
                robot.intakeServo.setPower(0);
            }

/*
            if (gamepad2.y) { //if button a pressed
                // Extend liftArm
                robot.liftArm.setPower(0.8);
                sleep(300);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);
            }
            if (gamepad2.a) { //if button a pressed
                // Retract liftArm
                robot.liftArm.setPower(-1.0);
                sleep(300);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);
            }

/*
            if(gamepad1.right_trigger > 0.7){
                robot.airplaneLauncher.setPosition(1.0);
            }



//grabber
            if (gamepad2.left_trigger > 0.5) {
                robot.grabServoLeft.setPosition(1.0); // open

            } else if (gamepad2.left_bumper) {
                robot.grabServoLeft.setPosition(0.0); // close


            }

            if (gamepad2.right_trigger > 0.5) {

                robot.grabServoRight.setPosition(0.0); // open
            } else if (gamepad2.right_bumper) {

                robot.grabServoRight.setPosition(1.0); // close
            }


//tilt arm
            if (gamepad2.left_stick_y > 0.7) {
                robot.liftHex.setPower(-0.5);
                robot.setDrivePower(vertical + turn - horizontal, vertical - turn + horizontal, vertical + turn + horizontal, vertical - turn - horizontal);
                //liftHexArm(-100, 0.6, 1000);

            }

            else if (gamepad2.left_stick_y < -0.7) {
                robot.liftHex.setPower(0.3);
                robot.setDrivePower(vertical + turn - horizontal, vertical - turn + horizontal, vertical + turn + horizontal, vertical - turn - horizontal);
                //liftHexArm(100, 0.6, 1000);


            }

            else {
                robot.liftHex.setPower(0);
            }



// tilt servo
            if (gamepad2.right_stick_y > 0.7) {
                robot.tiltServoLeft.setPosition(1.0);

            } else if (gamepad2.right_stick_y < -0.7) {
                robot.tiltServoLeft.setPosition(0);
            }


        }


        //emergency releases
    }

    void liftHexArm(int ticks, double power, long timeOutMills) {
        long timeCurrent, timeBegin;
        timeBegin = timeCurrent = System.currentTimeMillis();

        robot.liftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftHex.setTargetPosition(ticks);
        robot.liftHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftHex.setPower(power);
        while(opModeIsActive()
                && robot.liftHex.isBusy()
                && (timeCurrent - timeBegin) < timeOutMills) {
            timeCurrent = System.currentTimeMillis();
        }

    }

    private void TiltLiftOne ( double crankPowerBegin, int crankTimeMs, double crankPowerEnd,
        double liftPowerBegin, int liftTimeMs, double liftPowerEnd){
            //tilt the lift to be upright
            robot.liftHex.setPower(crankPowerBegin);   //set motor power
            sleep(crankTimeMs);          // let motor run for some time seconds.
            robot.liftHex.setPower(crankPowerEnd);   //set lower motor power to maintain the position

            // Extend liftArm
            robot.liftArm.setPower(liftPowerBegin);
            sleep(liftTimeMs);             // let motor run for some time seconds.
            robot.liftArm.setPower(liftPowerEnd);
        }
*/
        }

    }
}
