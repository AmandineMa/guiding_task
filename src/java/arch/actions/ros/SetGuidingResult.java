package arch.actions.ros;

import arch.agarch.SupervisorAgArch;
import jason.asSemantics.ActionExec;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import ros.RosNodeGuiding;

public class SetGuidingResult extends AbstractAction {
	
	private SupervisorAgArch rosAgArch = (SupervisorAgArch) super.rosAgArch;

	public SetGuidingResult(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}

	@Override
	public void execute() {
		logger.info("cancel dialogue goal when goal over");
		((RosNodeGuiding) rosnode).cancelDialogueInformGoal();
		((RosNodeGuiding) rosnode).cancelDialogueQueryGoal();
		String success = removeQuotes(actionExec.getActionTerm().getTerm(0).toString());
		String id = removeQuotes(actionExec.getActionTerm().getTerm(1).toString());
		if(!success.equals("preempted"))
			((RosNodeGuiding) rosnode).setTaskResult(success, id);
		logger.info("goal result : "+success);
		if(rosAgArch.getCurrentGoal() != null && rosAgArch.getCurrentGoal().equals(id)) {
			rosAgArch.setCurrentGoal(null);
		}
		actionExec.setResult(true);

	}

}
