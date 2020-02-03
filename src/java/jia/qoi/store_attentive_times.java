package jia.qoi;

import arch.agarch.InteractAgArch;
import jason.JasonException;
import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Term;

public class store_attentive_times extends DefaultInternalAction {
	
	@Override public int getMinArgs() {
        return 1;
    }
    @Override public int getMaxArgs() {
        return 1;
    }
	
	@Override 
	protected void checkArguments(Term[] args) throws JasonException {
        super.checkArguments(args); // check number of arguments
        if (!(args[0] instanceof NumberTermImpl))
            throw JasonException.createWrongArgument(this,"first argument must be a number");
    }

	@Override
	public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
		InteractAgArch arch = (InteractAgArch)ts.getUserAgArch();
		arch.attentive_times_add_value(arch.getRosTimeMilliSeconds());
		return true;
	}

	

}
