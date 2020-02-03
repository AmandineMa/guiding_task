package arch.agarch;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.logging.Logger;

import com.google.common.collect.Multimap;

import arch.actions.ActionFactoryImpl;
import jason.RevisionFailedException;
import jason.asSemantics.ActionExec;
import jason.asSemantics.Message;
import jason.asSemantics.Unifier;
import jason.asSyntax.Atom;
import jason.asSyntax.Literal;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Structure;
import rjs.agent.TimeBB;
import rjs.arch.actions.Action;
import rjs.arch.agarch.AbstractROSAgArch;
import rjs.utils.SimpleFact;
import rjs.utils.Tools;
import rjs.utils.XYLineChart_AWT;
import ros.RosNodeGuiding;

public class AgArchGuiding extends AbstractROSAgArch {

	protected class UpdateTimeBB extends TimeBB{
		@Override
		   public boolean add(Literal l) {
				if(contains(l) != null) {
					remove(l);
				}
				return super.add(l);
		    }
	}
	
	protected class QoIDB extends LinkedList<Literal> {

		@Override
		public boolean add(Literal e) {
			Structure time = new Structure("add_time");
			Double t = getConnectedNode().getCurrentTime().toSeconds() * 1000;
			time.addTerm(new NumberTermImpl(t.longValue()));
			e.addAnnot(time);
			return super.add(e);
		}
	}
	
	
	static protected XYLineChart_AWT display = new XYLineChart_AWT("QoI", "QoI data");

	protected Logger logger = Logger.getLogger(AgArchGuiding.class.getName());
	protected int percept_id = -1;
	
	
	public AgArchGuiding() {
		super();
	}
	
	
	@Override
	public void init() {
		super.init();
		setActionFactory(new ActionFactoryImpl());
	}


	@Override
	public Collection<Literal> perceive() {
		Collection<Literal> l = new ArrayList<Literal>();
		if(rosnode != null) {
//			if(percept_id != m_rosnode.getPercept_id()) {
				Multimap<String,SimpleFact> mm = ((RosNodeGuiding) rosnode).getPerceptions();
				synchronized (mm) {
					Collection<SimpleFact> perceptions = new ArrayList<SimpleFact>(mm.get("\""+getAgName()+"\""));
					if(perceptions != null) {
						for(SimpleFact percept : perceptions) {
							if(percept.getObject().isEmpty()) {
								l.add(Literal.parseLiteral(percept.getPredicate()));
							}else {
								l.add(Literal.parseLiteral(percept.getPredicate()+"("+percept.getObject()+")"));
							}
						}
					}
				}
				percept_id = ((RosNodeGuiding) rosnode).getPercept_id();
//			}
		}
		return l;
		
	}
	
	public class AgRunnable implements Runnable {
		
		private ActionExec action;
		
		public AgRunnable(AgArchGuiding rosAgArch, ActionExec action) {
			this.action = action;
		}
		
		@Override
		public void run() {
			String action_name = action.getActionTerm().getFunctor();
			Message msg = new Message("tell", getAgName(), "supervisor", "action_started(" + action_name + ")");
			String tmp_task_id = "";
			if (action.getIntention().getBottom().getTrigger().getLiteral().getTerms() != null)
				tmp_task_id = action.getIntention().getBottom().getTrigger().getLiteral().getTerm(0).toString();
			taskID = tmp_task_id;
			try {
				sendMsg(msg);
			} catch (Exception e) {
				Tools.getStackTrace(e);
			}
			Action actionExecutable = actionFactory.createAction(action, AgArchGuiding.this);
			if(action != null) {
				actionExecutable.execute();
				if(actionExecutable.isSync())
					actionExecuted(action);
			} else {
				action.setResult(false);
				action.setFailureReason(new Atom("act_not_found"), "no action " + action_name + " is implemented");
				actionExecuted(action);
			}
		}
	}
	
	@Override
	public void act(final ActionExec action) {
		executor.execute(new AgRunnable(this, action));
	}
	
	protected String taskID = "";
	
	public void addBelief(String bel) {
		try {
			
			getTS().getAg().addBel(Literal.parseLiteral(bel+("".equals(taskID)?"":"[" + taskID + "]")));
		} catch (RevisionFailedException e) {
			logger.info(Tools.getStackTrace(e));
		}
	}
	
	public void removeBelief(String bel) {
		try {
			
			getTS().getAg().abolish(Literal.parseLiteral(bel), new Unifier());
		} catch (RevisionFailedException e) {
			logger.info(Tools.getStackTrace(e));
		}
	}
	
	public String getTaskID() {
		return taskID;
	}
	
	public void saveChart() {
		display.saveChart();
	}

	
}
