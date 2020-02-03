package arch.actions.robot;

import jason.asSemantics.ActionExec;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import ros.RosNodeGuiding;

public class Engage extends AbstractAction {
	
	public Engage(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}

	@Override
	public void execute() {
		String human_id = actionExec.getActionTerm().getTerm(0).toString();
		((RosNodeGuiding) rosnode).callEngageAS(human_id);
		actionExec.setResult(true);
	}

}
