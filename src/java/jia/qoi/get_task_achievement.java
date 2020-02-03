package jia.qoi;

import arch.agarch.RobotAgArch;
import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Term;

public class get_task_achievement extends DefaultInternalAction {

	private static final long serialVersionUID = 1L;

	@Override
	public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
		return un.unifies(args[0], new NumberTermImpl(((RobotAgArch) ts.getUserAgArch()).task_achievement()));
	}
	
	

}
