package jia.qoi;

import java.io.File;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedList;

import arch.agarch.InteractAgArch;
import arch.agarch.RobotAgArch;
import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.Literal;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Term;

public class qoi_to_file  extends DefaultInternalAction {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	@Override public int getMinArgs() {
		return 3;
	}
	@Override public int getMaxArgs() {
		return 3;
	}

	@Override
	public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
		checkArguments(args);
		
		String type = args[0].toString();	
    	String name = args[1].toString().replaceAll("^\"|\"$", "");
    	String id = args[2].toString();
    	LinkedList<String> s_list = new LinkedList<String>();
    	
    	int counter = 1;

    	File  f = new File("log/qoi");
    	if(!f.exists()){
    		f.mkdirs();
    	}
    	String file_name = name;
    	Path path = Paths.get("log/qoi/"+name);
    	while(Files.exists(path)){
    		file_name = name+"_"+counter;
    	    path = Paths.get("log/qoi/"+file_name);
    	    counter++;
    	}
    	
    	if(type.equals("task")) {
    		Iterator<Literal> iteTask = ((RobotAgArch) ts.getUserAgArch()).getTaskQoI(id).iterator();
	    	while(iteTask.hasNext()) {
	    		s_list.add(iteTask.next().toString());
	    	}
	    	s_list.add("----------------------ACTIONS----------------------");
	    	Iterator<Literal> iteAction = ((RobotAgArch) ts.getUserAgArch()).getActionQoI(id).iterator();
	    	while(iteAction.hasNext()) {
	    		s_list.add(iteAction.next().toString());
	    	}
    	} else if(type.equals("session")) {
    		((InteractAgArch) ts.getUserAgArch()).saveChart();
    		Iterator<Literal> iteSession = ((InteractAgArch) ts.getUserAgArch()).getSessionQoI(id).iterator();
        	while(iteSession.hasNext()) {
        		Literal l = iteSession.next();
        		s_list.add(l.toString());
        	}
    	}
    	((RobotAgArch) ts.getUserAgArch()).out_task.close();
    	((RobotAgArch) ts.getUserAgArch()).out_action.close();
    	((RobotAgArch) ts.getUserAgArch()).saveChart();
    	Files.write(path, s_list, StandardCharsets.UTF_8);
    	
    	return true;
    }
	
	class LiteralTimeComparator implements Comparator<Literal> {
        @Override
        public int compare(Literal a, Literal b) {
            return (int) (getAddTime(a) - getAddTime(b)) ;
        }
    }
    
    public double getAddTime(Literal l) {
    	double result = 0;
        Literal addTime = l.getAnnot("add_time");
        if(addTime != null) {
            NumberTermImpl time = (NumberTermImpl) addTime.getTerm(0);
            if(time != null) {
                result = time.solve();
            }
        }
        return result;
    }
}
