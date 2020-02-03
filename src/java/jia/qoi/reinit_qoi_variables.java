package jia.qoi;

import arch.agarch.RobotAgArch;
import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.Term;

public class reinit_qoi_variables extends DefaultInternalAction {

	private static final long serialVersionUID = 1L;

	@Override
	public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
		((RobotAgArch) ts.getUserAgArch()).reinit_steps_number();
		((RobotAgArch) ts.getUserAgArch()).reinit_step();
		((RobotAgArch) ts.getUserAgArch()).reinit_qoi_variables();
		return true;
	}
	
	

}
