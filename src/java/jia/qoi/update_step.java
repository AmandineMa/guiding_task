package jia.qoi;

import arch.agarch.RobotAgArch;
import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.Term;

public class update_step extends DefaultInternalAction {

	private static final long serialVersionUID = 1L;

	@Override
	public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
		String value = ((Term) args[0]).toString();
		switch(value) {
		case "reinit":
			((RobotAgArch) ts.getUserAgArch()).reinit_step();
			break;
		case "increment":
			((RobotAgArch) ts.getUserAgArch()).increment_step();
			break;
		default:
			break;
		}
		return true;
	}
	
	

}
