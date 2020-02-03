package arch.actions.robot;

import java.util.HashMap;
import java.util.Map;

import org.ros.rosjava.tf.Transform;
import org.ros.rosjava.tf.TransformTree;

import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import pepper_resources_synchronizer_msgs.MetaStateMachineRegisterResponse;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import ros.RosNodeGuiding;

public class FaceHuman extends AbstractAction {
	
	public FaceHuman(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}

	@Override
	public void execute() {
		String frame = removeQuotes(actionExec.getActionTerm().getTerm(0).toString());

		TransformTree tfTree = rosAgArch.getTfTree();
		Transform transform;
		String frame1 = "base_link";
		if (tfTree.canTransform(frame1, frame)) {
			transform = tfTree.lookupMostRecent(frame1, frame);
			if(transform != null) {
				float d = (float) Math.atan2(transform.translation.y, transform.translation.x);

				Map<String, Object> parameters = new HashMap<String, Object>();
				parameters.put("statemachinepepperbasemanager", ((RosNodeGuiding) rosnode).buildStateMachinePepperBaseManager(actionName, (float) d));
				parameters.put("header", ((RosNodeGuiding) rosnode).buildMetaHeader());

				MetaStateMachineRegisterResponse faceResp = rosnode.callSyncService("pepper_synchro", parameters);
				actionExec.setResult(true);
				if(faceResp == null) {
					actionExec.setFailureReason(new Atom("cannot_face_human"), "Service Failure, face human failed for " + frame);
					actionExec.setResult(false);
				} 
			}else {
				actionExec.setResult(false);
				actionExec.setFailureReason(new Atom("cannot_face_human"), "Null transform, face human failed for " + frame);
			}

		} else {
			actionExec.setResult(false);
			actionExec.setFailureReason(new Atom("cannot_face_human"), "Cannot Transform, face human failed for " + frame);
		}
	}

}
