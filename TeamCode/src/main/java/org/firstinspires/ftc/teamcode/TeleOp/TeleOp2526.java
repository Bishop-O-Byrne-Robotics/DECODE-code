package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "TeleOp2526")
public class TeleOp2526 extends LinearOpMode {

    //Variables
    double speedCoefficient = 0.75;
    double turningCoefficient = 0.5;

    private SampleMecanumDrive drive;

    private boolean isForward = false;

    DcMotor intakeRoller;


    @Override
    public void runOpMode() throws InterruptedException {

        intakeRoller = hardwareMap.get(DcMotor.class, "Goonmaster67");
        intakeRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
                //orienting the robot for field centric
                if(!isForward){
                    orienting(gamepad1);
                    telemetry.update();
                    if (gamepad1.y) {
                        drive.setPoseEstimate(new Pose2d(
                                drive.getPoseEstimate().getX(),
                                drive.getPoseEstimate().getY(),
                                0 // reset heading
                        ));
                        isForward = true;
                    }
                    if (gamepad1.b){
                        intakeRoller.setPower(0.5);
                    } else{
                        intakeRoller.setPower(0);
                    }
                    telemetry.update();
                }

                else{
                    fieldCentricDrive(gamepad1);
                }
        }
    }
    public void fieldCentricDrive(Gamepad gamepad){

        //speed boost when holding the "a" button
        if(gamepad.a){
            speedCoefficient = 0.9;
            turningCoefficient = 0.9;
        }
        else{
            speedCoefficient = 0.7;
            turningCoefficient = 0.5;
        }

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();


        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad.left_stick_y * speedCoefficient,
                -gamepad.left_stick_x * speedCoefficient
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad.right_stick_x * turningCoefficient
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();

        telemetry.addData("Mode", "Field-Centric");
        telemetry.addData("X", poseEstimate.getX());
        telemetry.addData("Y", poseEstimate.getY());
        telemetry.addData("Heading (radians)", poseEstimate.getHeading());
        telemetry.addData("Heading (degrees)", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.update();
    }


    public void orienting(Gamepad gamepad) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y * 0.5,
                        -gamepad.left_stick_x * 0.5,
                        -gamepad.right_stick_x * 0.4
                )
        );
        drive.update();
    }
}
