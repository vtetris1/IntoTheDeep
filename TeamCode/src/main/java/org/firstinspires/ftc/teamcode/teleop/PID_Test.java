package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware2;

//test
@Config
@TeleOp(name = "PID_Test")

public class PID_Test extends LinearOpMode {
    private static double integralSum = 0;
    private static double Kp = 0;
    private static double Ki = 0;
    private static double Kd = 0;
    private static double Kf = 0;
    private static double lastError = 0;
    private static double target_position = 100;

    ElapsedTime timer = new ElapsedTime();




    RobotHardware2 robot = new RobotHardware2();

    private int sleepMs1 = 0;
//
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public  void runOpMode() throws InterruptedException {

        robot.init2(hardwareMap);

        // robot.liftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.liftHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // robot.liftHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {


            double Power = pidControl(target_position, robot.pivotMotor.getVelocity());
            robot.pivotMotor.setPower(Power);

            telemetry.addLine(String.format(" pivotMotor: %d ",
                    robot.pivotMotor.getCurrentPosition()
            ));
            telemetry.update();
/*

            while (gamepad2.left_stick_y > 0.7) {
                robot.pivotMotor.setPower(Power);
                sleep(15);
            }

            while (gamepad2.left_stick_y < -0.7) {
                robot.pivotMotor.setPower(-Power);
                sleep(15);
            }




            if (gamepad2.left_stick_y > 0.7) {
                robot.pivotMotor.setPower(0.4);
            }

            else if (gamepad2.left_stick_y < -0.7){
                robot.pivotMotor.setPower(-0.4);
            }

            else {
                robot.pivotMotor.setPower(0);
            }
*/
            // 0.7      1                    -0.7

//make sure one of the directions is correct/reversed


            //spin

            //tilt bucket





/*           //lift arm start
            if (gamepad2.a) { //if button a pressed
                robot.intakeServo.setPosition(1);
            }

            else if (gamepad2.y) {
                robot.intakeServo.setPosition(0);
            }

            else{
                robot.intakeServo.setPosition(0.5);
            }

*/
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

    private double pidControl(double refrence,double state) {
        double error  = refrence - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp + (derivative * Kd) + (integralSum * Ki) + (refrence * Kf));

        return output;
    }


}
