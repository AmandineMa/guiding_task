package jia.robot;

import java.util.Iterator;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.InternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.ListTerm;
import jason.asSyntax.ListTermImpl;
import jason.asSyntax.StringTermImpl;
import jason.asSyntax.Term;
import jason.asSyntax.VarTerm;

public class verba_name extends DefaultInternalAction {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
    @SuppressWarnings("rawtypes")
	@Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
    	Term[] args_i = new Term[3];
		args_i[0] = new StringTermImpl("getName");
		args_i[1] = null;
		args_i[2] = new VarTerm("Name");
		
		Term[] args_e = new Term[3];
		args_e[0] = new StringTermImpl("exist");
		args_e[1] = null;
		args_e[2] = new VarTerm("Bool");
		
		Term[] args_d = new Term[3];
		args_d[0] = new StringTermImpl("getDown");
		args_d[1] = null;
		args_d[2] = new VarTerm("Down");
		
		Term[] args_t = new Term[3];
		args_t[0] = new StringTermImpl("getType");
		args_t[1] = null;
		args_t[2] = new VarTerm("Type");
		
		Unifier u_i = new Unifier();
		
		Class iaclass_wi = Class.forName("jia.robot.word_individual");
		InternalAction ia_wi = (InternalAction)iaclass_wi.newInstance();
		Class iaclass_wc = Class.forName("jia.robot.word_class");
		InternalAction ia_wc = (InternalAction)iaclass_wc.newInstance();
		
    	if(args[0].isList()) {
    		ListTerm l = (ListTerm) args[0];
    		Iterator<Term> i = l.iterator();
    		ListTermImpl name_list = new ListTermImpl();
    		while(i.hasNext()) {
    			args_i[1] = i.next();
    			args_e[1] = args_i[1];
    			args_d[1] = args_i[1];
    			args_t[1] = args_i[1];
    			u_i = new Unifier();
    			if((boolean) ia_wi.execute(ts, u_i, args_e)) {
    				ia_wi.execute(ts, u_i, args_i);
    				name_list.add(u_i.get("Name"));
    			}else if((boolean) ia_wc.execute(ts, u_i, args_d) && !u_i.get("Down").isList()) {
    				ia_wi.execute(ts, u_i, args_t);
    				args_i[1] = u_i.get("Type");
    				ia_wi.execute(ts, u_i, args_i);
    				name_list.add(u_i.get("Name"));
    			}else if((boolean) ia_wc.execute(ts, u_i, args_e)) {
    				ia_wc.execute(ts, u_i, args_i);
    				name_list.add(u_i.get("Name"));
    			}
    		}
    		return un.unifies(args[1], name_list);	
    	}else if(args[0].isString() || args[0].isAtom()) {
    		args_i[1] = args[0];
    		if((boolean) ia_wi.execute(ts, u_i, args_i)) {
				ia_wi.execute(ts, u_i, args_i);
			}else if((boolean) ia_wc.execute(ts, u_i, args_i)) {
				ia_wc.execute(ts, u_i, args_i);
			}
    		return un.unifies(args[1], u_i.get("Name"));	
    	}else {
    		return false;
    	}
    	
    }
    
    void sleep(long msec) {
		try {
			Thread.sleep(msec);
		} catch (InterruptedException ex) {
		}
	}



}

