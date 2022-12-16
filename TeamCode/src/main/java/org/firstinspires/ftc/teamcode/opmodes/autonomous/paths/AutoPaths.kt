package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.GlobalConfig.PipelineResult
import org.firstinspires.ftc.teamcode.GlobalConfig.PipelineResult.*
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.Bot
import kotlin.math.PI
import kotlin.math.sqrt

class AutoPaths(val opMode: LinearOpMode) {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart

    sealed class AutoPathElement(open val name: String) {
        class Path(override val name: String, val trajectory: Trajectory) : AutoPathElement(name)

        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit) : AutoPathElement(name)
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }


    val Double.toRadAS get() = (if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) this.toRadians else - this.toRadians)
    val Double.toRadians get() = (Math.toRadians(this))
    val Int.toRadians get() = (Math.toRadians(this.toDouble()))
    val Int.toRadAS get() = (this.toDouble().toRadAS)

    enum class AutoType{
        PARK,
        DELIVERY
    }

    fun pose2d(x: Double, y: Double, h: Double): Pose2d{
        return Pose2d(x, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) y else -y, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else -h)
    }
    fun pose2d(v: Vector2d, h: Double): Pose2d{
        return Pose2d(v.x, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) v.y else -v.y, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else -h)
    }
    fun vector2d(x: Double, y: Double): Vector2d {
        return Vector2d(x, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) y else -y)
    }

    val bypassSize = 10

    fun bypass(startPose: Pose2d, endPose: Pose2d): Pose2d{
        val dif = endPose.minus(startPose).vec()
        return Pose2d(startPose.vec().plus(dif.div(dif.norm() * bypassSize)), startPose.heading)
    }
    fun bypassVector(startVec: Vector2d, endVec: Vector2d): Vector2d{
        val dif = endVec.minus(startVec)
        return startVec.plus(dif.div(dif.norm() * bypassSize))
    }
    fun bypassStraight(startPose: Pose2d, angle: Double): Pose2d{
        return Pose2d(startPose.vec().plus(startPose.vec().rotated(angle - 90)), startPose.heading)
    }

    val globalConfig: GlobalConfig = GlobalConfig()

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner
    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()

    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path {
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    //Probably won't be used, but here just in case
    fun makeAction(name: String, action: () -> Unit): AutoPathElement.Action {
        return AutoPathElement.Action(name, action)
        //Redundant but conforms to naming scheme
    }

    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit) : MarkerCallback {
        override fun onMarkerReached() = func()
    }
    //TODO: solve this
//    private fun turn(from: Double, to: Double): AutoPathElement.Action {
//        return AutoPathElement.Action("Turn from ${Math.toDegrees(from).roundToInt()}deg" +
//                "to ${Math.toDegrees(to).roundToInt()}deg") {
//            bot.roadRunner.turn(to - from)
//        }
//    }
    val autoType = AutoType.DELIVERY

    //Insert pose/vector vals here

    private val epsilon = 2.0

    private val trackWidth = 16.0

    private val startPose = pose2d(-36.0, -72.0 + trackWidth / 2, 0.0)
    private val intakePose = pose2d(-12 - trackWidth / 2, 50.0, - PI)
    private val deliveryPose = pose2d(- trackWidth * sqrt(2.0) / 4 - epsilon,
        - 24.0 - trackWidth * sqrt(2.0) / 4 - epsilon,
        PI / 4)
    private val parkingPose = mapOf(
        ONE to vector2d(- 36.0, - 12.0),
        TWO to vector2d(- 36.0, - 36.0),
        THREE to vector2d(- 36.0, - 60.0)
    )

    //                                                                  ===================================================

    //example
    // private val dropSecondWobble = mapOf(
    //            0 to Pose2d(-4.2 + 1.5, -48.0 - 3.056 + 1f, (-90.0 + 30.268).toRadians),
    //            1 to Pose2d(24.0 - 9.45428 + 3f, -24.0 - 25.16465, (102.4 - 90.0).toRadians),
    //            4 to Pose2d(48 - 5.1, -48.0 - 3.0556 - 3f, (-90.0 + 30.268).toRadians)
    //    )

    //TODO: Make Trajectories in trajectorySets

    // was dropFreight
    private val dropCone = listOf(
        makeAction("run intake"){
//                bot.intake.run()
        },
        makePath("drive to deliver",
            drive.trajectoryBuilder(startPose, 0.0)
                .splineToSplineHeading(deliveryPose, PI / 4)
                .build()),
        makeAction("deliver cone"){
//                bot.outtake.extend();
        }
    )

    //TODO: Add intake trajectory

    fun getLastPose(paths: List<AutoPathElement>): Pose2d {
        for(i in paths.reversed()){
            if(i is AutoPathElement.Path){
                return i.trajectory.end()
            }
        }
        return startPose
    }

    fun park(result: PipelineResult): List<AutoPathElement>{
        return run{
            listOf(
                makePath("drive into signal zone",
                    drive.trajectoryBuilder(lastPosition, PI / 4)
                        .splineTo(parkingPose[result]!!, 0.0)
                        .build()
                )
            )
        }
    }

    fun cycle(result: PipelineResult): List<AutoPathElement> {
        return run {
            dropCone!! + listOf(
                makePath("drive into signal zone",
                    drive.trajectoryBuilder(lastPosition, PI / 4)
                        .splineTo(parkingPose[result]!!, 0.0)
                        .build()
                )
            )
        }
    }

    private val trajectorySets: Map<AutoType, Map<PipelineResult, List<AutoPathElement>>> = mapOf(
        AutoType.PARK to mapOf(
            ONE to park(ONE),
            THREE to park(THREE),
            TWO to park(TWO)
        ),
        AutoType.DELIVERY to mapOf(
            ONE to cycle(ONE),
            THREE to cycle(THREE),
            TWO to cycle(TWO)
        )
    )

    //end paste  ==========================================================================

    fun createTrajectory(pipelineResult: PipelineResult): ArrayList<AutoPathElement> {
        val list = ArrayList<AutoPathElement>()

        for (trajectory in trajectorySets[autoType]!![pipelineResult]!!) {
            list.add(trajectory)
        }

        return list
    }
}