//package org.firstinspires.ftc.teamcode.AutoOp;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Autonomous.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.Autonomous.trajectorysequence.TrajectorySequence;
//
//
//@Autonomous(group = "drive")
//public class Auto2425 extends LinearOpMode {
//
//    DcMotor horizontalSlide;
//    DcMotor verticalSlide;
//
//    Servo armRight;
//    Servo armLeft;
//    public double armPosition;
//    public static double armServoDown = 0.56;
//    public static double armServoHover = armServoDown - 0.02;
//    public static double armServoUp = 0.05;
//
//    Servo basketServo;
//    public static double basketServoOpen = 0.735;
//    public static double basketServoClosed = 1;
//
//    Servo clawServo;
//    public static double clawServoOpen = 0.64;
//    public static double clawServoClosed = 1;
//
//    Servo clawRotate;
//    public static double clawRotateDropOff = 1.0/3.0;
//
//    Servo clawPivot;
//    public static double clawPivotUp = 0.67;
//    public static double clawPivotDown = 0.15;
//
//    double bucketPoseX = -59;
//    double bucketPoseY = -59;
//    double bucketPoseDeg = Math.toRadians(45);
//
//    ElapsedTime timer = new ElapsedTime();
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        verticalSlide = hardwareMap.get(DcMotor.class, "verticalSlide");
//        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");
//
//        armLeft = hardwareMap.get(Servo.class, "armLeft");
//        armRight = hardwareMap.get(Servo.class, "armRight");
//        basketServo = hardwareMap.get(Servo.class,"basketServo");
//        clawServo = hardwareMap.get(Servo.class,"clawServo");
//        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
//        clawPivot = hardwareMap.get(Servo.class, "clawPivot");
//
//        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
//        verticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        drive.setPoseEstimate(new Pose2d(-32.40, -63.38, Math.toRadians(90.00)));
//
//        TrajectoryVelocityConstraint slowVel = drive.getVelocityConstraint(20, 2, 12.87);
//        TrajectoryAccelerationConstraint slowAcc = drive.getAccelerationConstraint(20);
//
//        TrajectoryVelocityConstraint fastVel = drive.getVelocityConstraint(45, 4, 12.87);
//        TrajectoryAccelerationConstraint fastAcc = drive.getAccelerationConstraint(45);
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//        TrajectorySequence preloaded = drive.trajectorySequenceBuilder(new Pose2d(-32.40, -63.38, Math.toRadians(90.00)))
//                .setConstraints(slowVel, slowAcc)
//                .lineToLinearHeading(new Pose2d(bucketPoseX, bucketPoseY, bucketPoseDeg))
//                .build();
//
//        TrajectorySequence firstSample = drive.trajectorySequenceBuilder(new Pose2d(bucketPoseX, bucketPoseY, bucketPoseDeg))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> verticalSlideTo(0))
//                .splineTo(new Vector2d(-49.97, -40.01), Math.toRadians(90.00))
//                .build();
//
//        TrajectorySequence dropFirstSample = drive.trajectorySequenceBuilder(new Pose2d(-49.97, -40.01, Math.toRadians(90.00)))
//                .setConstraints(slowVel, slowAcc)
//                .lineToLinearHeading(new Pose2d(bucketPoseX, bucketPoseY, bucketPoseDeg))
//                .build();
//
//        TrajectorySequence secondSample = drive.trajectorySequenceBuilder(new Pose2d(bucketPoseX, bucketPoseY, bucketPoseDeg))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> verticalSlideTo(0))
//                .lineToLinearHeading(new Pose2d(-59.74, -40.51, Math.toRadians(90.00)))
//                .build();
//
//        TrajectorySequence dropSecondSample = drive.trajectorySequenceBuilder(new Pose2d(-59.74, -40.51, Math.toRadians(90.00)))
//                .setConstraints(slowVel, slowAcc)
//                .lineToLinearHeading(new Pose2d(bucketPoseX, bucketPoseY, bucketPoseDeg))
//                .build();
//
//
//        TrajectorySequence thirdSample = drive.trajectorySequenceBuilder(new Pose2d(bucketPoseX, bucketPoseY, bucketPoseDeg))
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> verticalSlideTo(0))
//                .splineTo(new Vector2d(-55.67, -43.37), Math.toRadians(139.01))
//                .build();
//
//        TrajectorySequence dropThirdSample = drive.trajectorySequenceBuilder(new Pose2d(-55.92, -43.37, Math.toRadians(139.01)))
//                .setConstraints(slowVel, slowAcc)
//                .lineToLinearHeading(new Pose2d(bucketPoseX, bucketPoseY, bucketPoseDeg))
//                .build();
//
//        TrajectorySequence touchBar = drive.trajectorySequenceBuilder(new Pose2d(bucketPoseX, bucketPoseY, bucketPoseDeg))
//                .setConstraints(fastVel, fastAcc)
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> verticalSlideTo(1000))
//                .splineTo(new Vector2d(-44.49, -13.84), Math.toRadians(142.52))
//                .lineToLinearHeading(new Pose2d(-23.28, -9.02, Math.toRadians(178.70)))
//                .build();
//
//
//
//
//
////        TrajectorySequence moveFirstSample = drive.trajectorySequenceBuilder(new Pose2d(31.06, -39.18, Math.toRadians(70)))
////                .setVelConstraint(new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(25), new AngularVelocityConstraint(2))))
////                .lineToSplineHeading(new Pose2d(31.06, -49.18, Math.toRadians(310)))
////                .build();
//
//
////        TrajectorySequence dropOff = drive.trajectorySequenceBuilder(new Pose2d(39.52, -50.13, Math.toRadians(310)))
////                .resetConstraints()
////                .lineToLinearHeading(new Pose2d(3.73, -32.23, Math.toRadians(270)))
////                .build();
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        resetServos();
//
//        //drop off preloaded sample
//        verticalSlideTo(3000);
//        drive.followTrajectorySequence(preloaded);
//        dropSample();
//
//        //pickup first sample
//        drive.followTrajectorySequence(firstSample);
//        pickUpSample();
//        resetArmServo();
//
//        //drop off first sample
//        verticalSlideTo(3000);
//        drive.followTrajectorySequence(dropFirstSample);
//        dropSample();
//
//        //pickup second sample
//        drive.followTrajectorySequence(secondSample);
//        pickUpSample();
//        resetArmServo();
//
//        //drop off second sample
//        verticalSlideTo(3000);
//        drive.followTrajectorySequence(dropSecondSample);
//        dropSample();
//
//        //pickup third sample
//        horizontalSlideTo(1000);
//        clawRotate.setPosition(0.56);
//        drive.followTrajectorySequence(thirdSample);
//        pickUpSample();
//        resetArmServo();
//
//        //drop off third sample
//        verticalSlideTo(3000);
//        drive.followTrajectorySequence(dropThirdSample);
//        dropSample();
//
//        //level 1 ascent
//        drive.followTrajectorySequence(touchBar);
//        verticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalSlide.setPower(-0.35);
//
//
//
//    }
//
//
//
//
//    public void resetServos(){
//        armPosition = armServoHover;
//        moveArmServo();
//        basketServo.setPosition(basketServoClosed);
//        clawServo.setPosition(clawServoOpen);
//        clawRotate.setPosition(clawRotateDropOff);
//        clawPivot.setPosition(clawPivotDown);
//    }
//
//
//    public void moveArmServo() {
//        armRight.setPosition(armPosition);
//        armLeft.setPosition(1 - armRight.getPosition());
//    }
//
//    public void verticalSlideTo(int position){
//        verticalSlide.setTargetPosition(position);
//        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        verticalSlide.setPower(1);
//    }
//
//    public void horizontalSlideTo(int position){
//        horizontalSlide.setTargetPosition(position);
//        horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        horizontalSlide.setPower(1);
//    }
//
//    public void pickUpSample(){
//        armPosition = armServoDown;
//        moveArmServo();
//        clawServo.setPosition(clawServoClosed);
//        sleep(400);
//        armPosition = armServoUp-0.1;
//        moveArmServo();
//        clawPivot.setPosition(clawPivotUp);
//        clawRotate.setPosition(clawRotateDropOff);
//        horizontalSlideTo(50);
//        sleep(900);
//        armPosition = armServoUp;
//        moveArmServo();
//        clawServo.setPosition(clawServoOpen);
//    }
//
//    public void resetArmServo(){
//        sleep(400);
//        //resetting for next
//        armPosition = armServoHover;
//        moveArmServo();
//        clawPivot.setPosition(clawPivotDown);
//    }
//
//    public void dropSample(){
//        basketServo.setPosition(basketServoOpen);
//        sleep(800);
//        basketServo.setPosition(basketServoClosed);
//    }
//
//
//}
