package arch.actions.robot;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import actionlib_msgs.GoalStatus;
import geometry_msgs.PoseStamped;
import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import jason.asSyntax.ListTermImpl;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Term;
import move_base_msgs.MoveBaseActionResult;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import rjs.msg_srv_impl.PoseCustom;
import rjs.utils.Tools;
import ros.RosNodeGuiding;
import std_msgs.Header;

public class MoveTo extends AbstractAction {
	
	public MoveTo(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}

	@Override
	public void execute() {
		String frame = actionExec.getActionTerm().getTerm(0).toString();
		Iterator<Term> action_term_it = ((ListTermImpl) actionExec.getActionTerm().getTerm(1)).iterator();
		List<Double> pose_values = new ArrayList<>();
		while (action_term_it.hasNext()) {
			pose_values.add(((NumberTermImpl) action_term_it.next()).solve());
		}
		action_term_it = ((ListTermImpl) actionExec.getActionTerm().getTerm(2)).iterator();
		while (action_term_it.hasNext()) {
			pose_values.add(((NumberTermImpl) action_term_it.next()).solve());
		}
		PoseCustom pose = new PoseCustom(pose_values);
		PoseStamped pose_stamped = rosAgArch.createMessage(PoseStamped._TYPE);
		Header header = rosAgArch.createMessage(std_msgs.Header._TYPE);
		header.setFrameId(frame);
		pose_stamped.setHeader(header);
		pose_stamped.setPose(pose.getPose());
		((RosNodeGuiding) rosnode).callMoveToAS(pose_stamped);
		MoveBaseActionResult move_to_result;
		do {
			move_to_result = ((RosNodeGuiding) rosnode).getMoveToResult();
			Tools.sleep(200);
		} while (move_to_result == null);
		if (move_to_result.getStatus().getStatus() == GoalStatus.SUCCEEDED) {
			rosAgArch.addBelief("move_goal_reached");
			actionExec.setResult(true);
		} else {
			actionExec.setResult(false);
			actionExec.setFailureReason(new Atom("move_to_failed"), "");
		}

	}

}
