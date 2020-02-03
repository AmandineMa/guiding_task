package arch.actions.internal;

import jason.asSemantics.ActionExec;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;

public class SetParam extends AbstractAction {

	public SetParam(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}

	@Override
	public void execute() {
		String paramName = removeQuotes(actionExec.getActionTerm().getTerm(0).toString());
		boolean b = Boolean.parseBoolean(actionExec.getActionTerm().getTerm(1).toString());
		rosnode.getParameters().set(paramName, b);
		actionExec.setResult(true);
	}

}
