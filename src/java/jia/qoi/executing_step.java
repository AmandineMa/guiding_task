package jia.qoi;

import arch.agarch.RobotAgArch;
import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.Term;

public class executing_step extends DefaultInternalAction {

	private static final long serialVersionUID = 1L;

	@Override
	public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
		String step = ((Term) args[0]).toString();
		((RobotAgArch) ts.getUserAgArch()).set_on_going_step(step);
		return true;
	}
	
	

}
