// Internal action code for project supervisor

package jia.robot;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.List;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.Atom;
import jason.asSyntax.ListTermImpl;
import jason.asSyntax.Literal;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.StringTermImpl;
import jason.asSyntax.Term;
import rjs.arch.agarch.AbstractROSAgArch;

/** Handle only lists of Double/Integer **/

public class get_param extends DefaultInternalAction {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
    	try {
	    	String class_name = args[1].toString().replaceAll("^\"|\"$", "");
	    	String prefix = "java.lang.";
	    	if(class_name.equals("List"))
	    		prefix = "java.util.";
			Class<?> c = Class.forName(prefix+class_name);
			Method m = AbstractROSAgArch.getRosnode().getParameters().getClass().getMethod("get"+class_name, String.class);
			if(c.equals(Boolean.class)){
				Boolean p = (Boolean) m.invoke(AbstractROSAgArch.getRosnode().getParameters(), args[0].toString().replaceAll("^\"|\"$", ""));
				return un.unifies(args[2], new Atom(Literal.parseLiteral(p.toString())));
			}else if(c.equals(Double.class)) {
				Double p = (Double) m.invoke(AbstractROSAgArch.getRosnode().getParameters(), args[0].toString().replaceAll("^\"|\"$", ""));
				return un.unifies(args[2], new NumberTermImpl(p));
			}else if(c.equals(Integer.class)) {
				Integer p = (Integer) m.invoke(AbstractROSAgArch.getRosnode().getParameters(), args[0].toString().replaceAll("^\"|\"$", ""));
				return un.unifies(args[2], new NumberTermImpl(p));
			}else if(c.equals(String.class)) {
				String p = (String) m.invoke(AbstractROSAgArch.getRosnode().getParameters(), args[0].toString().replaceAll("^\"|\"$", ""));
				return un.unifies(args[2], new StringTermImpl(p));
			}else if(c.equals(List.class)) {
				List<?> p = (List<?>) m.invoke(AbstractROSAgArch.getRosnode().getParameters(), args[0].toString().replaceAll("^\"|\"$", ""));
				ListTermImpl list = new ListTermImpl();
				for(int i=0; i < p.size(); i++) {
					Double d = null;
					if(p.get(i) instanceof Integer)
						d = ((Integer) p.get(i)).doubleValue();
					else if(p.get(i) instanceof Double)
						d = (Double) p.get(i);
					if(d != null)
						list.add(new NumberTermImpl(d));
					else
						return false;
				}
				return un.unifies(args[2], list);
			}else {
				return false;
			}
    	}catch(InvocationTargetException e){
    		e.printStackTrace();
    		return false;
    	}
    }

}
