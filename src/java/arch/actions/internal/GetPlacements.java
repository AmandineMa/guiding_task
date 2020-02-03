package arch.actions.internal;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;
import org.ros.rosjava.tf.Transform;
import org.ros.rosjava.tf.TransformFactory;
import org.ros.rosjava.tf.TransformTree;
import org.ros.rosjava_geometry.Vector3;

import arch.agarch.RobotAgArch;
import jason.asSemantics.ActionExec;
import pointing_planner.PointingPlannerResponse;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import rjs.msg_srv_impl.PoseCustom;
import rjs.utils.Tools;

public class GetPlacements extends AbstractAction {

	public GetPlacements(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
	}

	@Override
	public void execute() {
		ArrayList<String> params = removeQuotes(actionExec.getActionTerm().getTerms());

		String target = params.get(0);
		String direction = params.get(1);
		String human = params.get(2);
		int tarIsDir = Integer.parseInt(params.get(3));

		ServiceResponseListener<PointingPlannerResponse> respListener = new ServiceResponseListener<PointingPlannerResponse>() {

			public void onFailure(RemoteException e) {
				PointingPlannerResponse placementsResp = rosAgArch.createMessage(PointingPlannerResponse._TYPE);
				placementsResp.setPointedLandmarks(new ArrayList<String>());
				pointPlan(placementsResp, human, tarIsDir, target, direction);
				actionExec.setResult(true);
				rosAgArch.actionExecuted(actionExec);
			}

			public void onSuccess(PointingPlannerResponse placements_resp) {
				pointPlan(placements_resp, human, tarIsDir, target, direction);
				actionExec.setResult(true);
				rosAgArch.actionExecuted(actionExec);
			}

		};

		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("targetlandmark", target);
		parameters.put("directionlandmark", direction);
		parameters.put("human", human);
		rosnode.callAsyncService("pointing_planner", respListener, parameters);
	}
	
	private void pointPlan(PointingPlannerResponse placementsResult, String human, int tarIsDir, String target, String direction) {
		if (placementsResult != null && !placementsResult.getPointedLandmarks().isEmpty()) {
			
			PoseCustom robotPose = new PoseCustom(placementsResult.getRobotPose().getPose());
			String rFrame = placementsResult.getRobotPose().getHeader().getFrameId();
			PoseCustom humanPose = new PoseCustom(placementsResult.getHumanPose().getPose());
			String hFrame = placementsResult.getHumanPose().getHeader().getFrameId();
			TransformTree tfTree = ((RobotAgArch) rosAgArch).getTfTree();
			Transform robotPoseNow;
			robotPoseNow = tfTree.lookupMostRecent("map", "base_footprint");
			if(robotPoseNow != null) {
				
				double rDistToNewPose = Math.hypot(
						robotPoseNow.translation.x - robotPose.getPosition().getX(),
						robotPoseNow.translation.y - robotPose.getPosition().getY());
				
				logger.info("robot dist to new pose :"+rDistToNewPose);
				if (rDistToNewPose > rosnode.getParameters().getDouble("guiding/tuning_param/robot_should_move_dist_th")) {

					rosAgArch.addBelief("robot_move(" + rFrame + ","+ robotPose.toString() + ")");

					Transform humanPoseNow = tfTree.lookupMostRecent("map", human);
					if(humanPoseNow != null) {
						double hDistToNewPose = Math.hypot(
								humanPoseNow.translation.x - robotPose.getPosition().getX(),
								humanPoseNow.translation.y - robotPose.getPosition().getY());

						logger.info("human pose now :"+humanPoseNow);
						logger.info("dist future robot pose and human now :"+hDistToNewPose);

						if (hDistToNewPose < rosnode.getParameters().getDouble("guiding/tuning_param/human_move_first_dist_th")) {
							geometry_msgs.Vector3 vectorMsg = rosAgArch.createMessage(geometry_msgs.Vector3._TYPE);
							
							computeHumanSide(TransformFactory.vector2msg(robotPoseNow.translation), 
											 TransformFactory.vector2msg(humanPoseNow.translation),
											 Vector3.fromPointMessage(humanPose.getPosition()).toVector3Message(vectorMsg));
						}
					}
				}else {

					rosAgArch.addBelief("robot_turn(" + rFrame + ", ["
							+ robotPoseNow.translation.x + ","+ robotPoseNow.translation.y + "," + robotPoseNow.translation.z+ "], ["  
							+ robotPose.getOrientation().getX()+ "," + robotPose.getOrientation().getY()+ ","
							+ robotPose.getOrientation().getZ()+ "," + robotPose.getOrientation().getW()+ "])");
				}
			}else {
				logger.info("robot pose now = null");
				rosAgArch.addBelief("robot_move(" + rFrame + "," + robotPose.toString() + ")");
			}
			
			rosAgArch.addBelief("robot_pose(" + rFrame + "," + robotPose.toString() + ")");
			rosAgArch.addBelief("human_pose(" + hFrame + "," + humanPose.toString() + ")");
			
			computeBeliefsLdToPoint(placementsResult.getPointedLandmarks().size(), placementsResult.getPointedLandmarks(), tarIsDir, target, direction);
			
			rosAgArch.addBelief("ld_to_point");
		} else {
			rosAgArch.addBelief("~ld_to_point");
		}
	}
	
	private void computeHumanSide(geometry_msgs.Vector3 robotPoseNow, geometry_msgs.Vector3 humanPoseNow, geometry_msgs.Vector3 humanPoseFuture) {
		String side;
		// isLeft from robot view then it is right from human view
		side = Tools.isLeft(robotPoseNow, humanPoseNow, humanPoseFuture) ? "right" : "left";
		rosAgArch.addBelief("human_first(" + side + ")");
	}
	
	private void computeBeliefsLdToPoint(int nbLdToPoint, List<String> ldToPoint, int tarIsDir, String target, String direction) {
		for (int i = 0; i < nbLdToPoint; i++) {
			String ld = ldToPoint.get(i);
			if (tarIsDir == 0) {
				if (ld.equals(target)) {
					rosAgArch.addBelief("target_to_point(\"" + ld + "\")");
				} else if (ld.equals(direction)) {
					rosAgArch.addBelief("dir_to_point(\"" + ld + "\")");
				}
			} else {
				if (ld.equals(target)) {
					rosAgArch.addBelief("dir_to_point(\"" + ld + "\")");
				}
			}
		}
	}

}
