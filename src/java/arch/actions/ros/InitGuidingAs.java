package arch.actions.ros;

import com.github.rosjava_actionlib.ActionServerListener;

import actionlib_msgs.GoalID;
import arch.agarch.SupervisorAgArch;
import guiding_as_msgs.taskActionGoal;
import jason.asSemantics.ActionExec;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import rjs.utils.Tools;
import ros.RosNodeGuiding;

public class InitGuidingAs extends AbstractAction {
	
	private SupervisorAgArch rosAgArch = (SupervisorAgArch) super.rosAgArch;

	public InitGuidingAs(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}

	@Override
	public void execute() {
		ActionServerListener<taskActionGoal> listener = new ActionServerListener<taskActionGoal>() {

			@Override
			public void goalReceived(taskActionGoal goal) {
				GoalID goalID = goal.getGoalId();
				if(goal.getGoalId().getId().isEmpty()) {
					rosAgArch.getGoalIDGenerator().generateID(goalID);
					goal.setGoalId(goalID);
				}
			}

			@Override
			public void cancelReceived(GoalID id) {
				rosAgArch.addBelief("cancelled(\""+id.getId()+"\")");
				logger.info("cancel dialogue goal when goal cancelled");
			}

			@Override
			public boolean acceptGoal(taskActionGoal goal) {
				logger.info("accept new goal :"+goal.getGoal().getPlaceFrame());
				if(rosAgArch.getCurrentGoal() != null) {
					logger.info("cancel dialogue goal when new goal received");
					rosAgArch.addBelief("preempted(\""+rosAgArch.getCurrentGoal()+"\")");
				}
				while(rosAgArch.getCurrentGoal() != null) {
					Tools.sleep(100);
				}
				rosAgArch.setCurrentGoal(goal.getGoalId().getId());
				String person = "\""+goal.getGoal().getPersonFrame()+"\"";
				if(rosnode.getParameters().getBoolean("guiding/dialogue/hwu"))
					person = person.replaceAll("human-", "");
				rosAgArch.addBelief("guiding_goal(\""+goal.getGoalId().getId()+"\",\"0\",\""+goal.getGoal().getPlaceFrame()+"\")");
				return true;
			}
		};
		((RosNodeGuiding) rosnode).setGuidingASListener(listener);
		actionExec.setResult(true);

	}

}
