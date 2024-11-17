package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
/*
@Autonomous(name="Gyro", group = "test")
public class Gyro extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // imu variables
    private YawPitchRollAngles lastAngles;
    private double currAngle = 0.0;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()){
/*            Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//            telemetry.addData("Orientation: ",orientation.firstAngle);
//            telemetry.update();
//        }

        turnTo(90);
        sleep(3000);


    }
    public void resetAngle(){
        lastAngles = robot.imu.getRobotYawPitchRollAngles();
        currAngle = 0;
    }

    public double getAngle(){
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();

        double deltaAngle = orientation.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

        if (deltaAngle>180){
            deltaAngle -= 360;
        }
        else if (deltaAngle<=-180){
            deltaAngle+=360;
        }

        currAngle+=deltaAngle;
        lastAngles = orientation;

        telemetry.addData("gyro", orientation.getYaw(AngleUnit.DEGREES));
        return currAngle;
    }
    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error)>2){
            //double motorPower = (error < 0 ? -0.3 : 0.3);
            double motorPower = Math.min(error*0.01,0.3);
            robot.setDrivePower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.setAllDrivePower(0);
    }
    public void turnTo(double degrees){
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();

        double error = degrees - orientation.getYaw(AngleUnit.DEGREES);

        if (error > 180){
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }
        while (opModeIsActive() && Math.abs(error)>10){
            YawPitchRollAngles currOrientation = robot.imu.getRobotYawPitchRollAngles();

            double motorPower = (error < 0 ? -0.35 : 0.35);
            robot.setDrivePower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - currOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("Orientation: ",currOrientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Error: ", Math.abs(error));

            telemetry.update();

        }

    }
}
*/