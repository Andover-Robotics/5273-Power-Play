package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.GlobalConfig.*
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


    val extensionDelay: Long = 750
    val Double.toRadAS get() = (if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) this.toRadians else - this.toRadians)
    val Double.toRadians get() = (Math.toRadians(this))
    val Int.toRadians get() = (Math.toRadians(this.toDouble()))
    val Int.toRadAS get() = (this.toDouble().toRadAS)

    enum class AutoType{
        PARK,
        DELIVERY
    }

    fun p2d(x: Double, y: Double, h: Double): Pose2d{
        return Pose2d(if (GlobalConfig.side == GlobalConfig.Side.AUDIENCE) x else - x, if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) y else -y, if (GlobalConfig.side == GlobalConfig.Side.AUDIENCE) (if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else -h) else 2 * Math.PI - (if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else -h))
    }

    fun p2d(v: Vector2d, h: Double): Pose2d{
        return Pose2d(if (GlobalConfig.side == GlobalConfig.Side.AUDIENCE) v.x else - v.x, if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) v.y else -v.y, if (GlobalConfig.side == GlobalConfig.Side.AUDIENCE) (if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else -h) else 2 * Math.PI - (if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else -h))
    }

    fun v2d(x: Double, y: Double): Vector2d {
        return Vector2d(if (GlobalConfig.side == GlobalConfig.Side.AUDIENCE) x else - x, if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) y else -y)
    }

    fun h2d(h: Double): Double {
        return if (side == GlobalConfig.Side.AUDIENCE) (if (alliance == GlobalConfig.Alliance.RED) h else PI-h) else 2 * Math.PI - (if (alliance == GlobalConfig.Alliance.RED) h else PI-h)
    }

    val bypassSize = 10

    fun bypass(startPose: Pose2d, endPose: Pose2d): Pose2d{
        val dif = endPose.minus(startPose).vec()
        return Pose2d(startPose.vec().plus(dif.div(dif.norm() * bypassSize)), startPose.heading)
    }
    fun bypassVec(startVec: Vector2d, endVec: Vector2d): Vector2d{
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

    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path {
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

    private val trackWidth = 15.0

    private var lastPosition = p2d(-36.0, -72.0 + trackWidth / 2, 0.0)

    //TODO: Fix coordinates(maybe X and Y axes are swapped?)

    private val intakePose = p2d( - 54.0 + trackWidth / 2, - 12.0, PI / 2)
    private val deliveryPose = p2d(- 24.0 - trackWidth * sqrt(2.0) / 4 - epsilon,
        - trackWidth * sqrt(2.0) / 4 - epsilon,
        7 * PI / 4)
    private val parkingPose = mapOf(
        ONE to v2d(- 12.0, - 36.0),
        TWO to v2d(- 36.0, - 36.0),
        THREE to v2d(- 60.0, - 36.0)
    )

    //                                                                  ===================================================

    //example
    // private val dropSecondWobble = mapOf(
    //            0 to Pose2d(-4.2 + 1.5, -48.0 - 3.056 + 1f, (-90.0 + 30.268).toRadians),
    //            1 to Pose2d(24.0 - 9.45428 + 3f, -24.0 - 25.16465, (102.4 - 90.0).toRadians),
    //            4 to Pose2d(48 - 5.1, -48.0 - 3.0556 - 3f, (-90.0 + 30.268).toRadians)
    //    )

    fun intakeFreight(): List<AutoPathElement> {
        println(lastPosition)

        val trajectoryList = listOf(AutoPathElement.Path("", drive.trajectoryBuilder(lastPosition, lastPosition.heading)
            .lineTo(Vector2d(lastPosition.x, intakePose.y))
            .build()),
            AutoPathElement.Path("", drive.trajectoryBuilder(Pose2d(lastPosition.x, intakePose.y, lastPosition.heading), lastPosition.heading)
                .lineToLinearHeading(intakePose)
                .build()))
        lastPosition = trajectoryList[1].trajectory.end()
        return trajectoryList
    }

    fun depositFreight(): List<AutoPathElement> {
        println(lastPosition)

        val trajectoryList = listOf(
            AutoPathElement.Path(
                "", drive.trajectoryBuilder(lastPosition, lastPosition.heading)
                    .lineTo(Vector2d(deliveryPose.x, lastPosition.y - 0.01))
                    .build()
            ),
            AutoPathElement.Path(
                "",
                drive.trajectoryBuilder(
                    Pose2d(
                        deliveryPose.x,
                        lastPosition.y - 0.01,
                        lastPosition.heading
                    ), lastPosition.heading
                )
                    .lineToLinearHeading(deliveryPose)
                    .build()
            )
        )

        lastPosition = trajectoryList[1].trajectory.end()
        return trajectoryList
    }

    /*

    fun getLastPose(paths: List<AutoPathElement>): Pose2d {
        for(i in paths.reversed()){
            if(i is AutoPathElement.Path){
                return i.trajectory.end()
            }
        }
        return startPose
    }

     */

    fun park(result: PipelineResult): List<AutoPathElement> {

        var reflected: Map<PipelineResult, PipelineResult>

        if ((alliance == Alliance.RED && side == Side.AUDIENCE) || !((alliance == GlobalConfig.Alliance.RED) || (side == GlobalConfig.Side.AUDIENCE)))
            reflected = mapOf(
                ONE to THREE,
                TWO to TWO,
                THREE to ONE
            )

        else
            reflected = mapOf(
                ONE to ONE,
                TWO to TWO,
                THREE to THREE
            )

        println(lastPosition)

        val trajectoryList = listOf(AutoPathElement.Path("", drive.trajectoryBuilder(lastPosition, lastPosition.heading)
            .lineTo(Vector2d(lastPosition.x, parkingPose[reflected[result]]!!.y - 0.01))
            .build()),
            AutoPathElement.Path("", drive.trajectoryBuilder(Pose2d(lastPosition.x, parkingPose[reflected[result]]!!.y - 0.01, lastPosition.heading), lastPosition.heading)
                .lineTo(parkingPose[reflected[result]]!!)
                .build()))

        lastPosition = Pose2d(parkingPose[reflected[result]]!!.x, parkingPose[reflected[result]]!!.y, lastPosition.heading)

        return trajectoryList

    }

    private val trajectorySets: Map<AutoType, Map<PipelineResult, List<AutoPathElement>>> = mapOf(

        AutoType.DELIVERY to mapOf(
            ONE to depositFreight() + intakeFreight() + depositFreight() + intakeFreight() + depositFreight() + park(ONE),
            THREE to depositFreight() + intakeFreight()+ depositFreight() + intakeFreight() + depositFreight() + park(TWO),
            TWO to depositFreight() + intakeFreight() + depositFreight() + intakeFreight() + depositFreight() + park(THREE)
        ),

        AutoType.PARK to mapOf(
            ONE to park(ONE),
            TWO to park(TWO),
            THREE to park(THREE)
        )

    )

    //end paste  ==========================================================================

    fun createTrajectory(pipelineResult: PipelineResult): ArrayList<AutoPathElement> {

        val list = ArrayList<AutoPathElement>();

        for (trajectory in trajectorySets[autoType]!![pipelineResult]!!) {
            list.add(trajectory)
        }

        return list

    }

}