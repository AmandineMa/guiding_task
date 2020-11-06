package arch.agarch;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.ros.message.MessageListener;
import org.ros.rosjava.tf.Transform;
import org.ros.rosjava.tf.TransformTree;

import deictic_gestures.LookAtStatus;
import deictic_gestures.PointAtStatus;
import jason.RevisionFailedException;
import jason.architecture.AgArch;
import jason.asSemantics.ActionExec;
import jason.asSemantics.Message;
import jason.asSyntax.ListTermImpl;
import jason.asSyntax.Literal;
import jason.asSyntax.NumberTermImpl;
import rjs.utils.QoI;
import rjs.utils.Tools;
import ros.RosNodeGuiding;
import std_msgs.Float64;

public class RobotAgArch extends AgArchGuiding {

	protected Logger logger = Logger.getLogger(RobotAgArch.class.getName());
	
	std_msgs.Header her;
	AgArch interact_arch = new InteractAgArch();
	double startTimeOngoingAction = -1;
	double startTimeOngoingStep = -1;
	HashMap<String,Double> actionsThreshold = new HashMap<String,Double>() {{
        put("speak", 10000.0);
        put("question", 8000.0);
        put("robot_move", 20000.0);
        put("come_closer", 30000.0);
        put("step", 20000.0);
    }};
	HashMap<String,Double> stepsThreshold = new HashMap<String,Double>() {{
        put("goal_nego", 30000.0);
        put("person_abilities", 15000.0);
        put("agents_at_right_place", 30000.0);
        put("target_explanation", 20000.0);
        put("direction_explanation", 30000.0);
        put("ask_landmark_seen", 20000.0);
        put("ask_understood", 20000.0);
    }};
	
	String onGoingAction = "";
	String onGoingStep = "";
	String speak;
	HashMap<String, QoIDB> actionsQoI = new HashMap<String, QoIDB>();
	HashMap<String, QoIDB> tasksQoI = new HashMap<String,  QoIDB>();
	String taskId = "";
	boolean inQuestion = false;
	int humanAnswer = 0;
	double onTimeTaskExecution = 1;
	double onTimeTaskExecutionPrev = 1;
	double distToGoal = 0;
	double decreasingSpeed = 1.5;
	float step = 0;
	ArrayList<Double> currentTaskActQoI = new ArrayList<Double>();
	private final ReadWriteLock startActionLock = new ReentrantReadWriteLock();
	
	double steps = 2;
	double taskQoIAverage = 0;
	double nbTaskQoI = 0;
	boolean firstTimeInTask = true;
	Double prevTime = 0.0;
	double monitorTimeAnswering = 0;
	boolean wasComingCloser = false;
	boolean wasStepping = false;
	boolean wasMoving = false;
	boolean wasWaiting = false;
	boolean newStep = false;
	boolean startAction = false;
	Double firstTtg = Double.MAX_VALUE;
	Double ttg = Double.MAX_VALUE;
	double moveStarted = 0;
	Double countDistComeCloser = 0.0;
	Double prevDistComeCloser = Double.MAX_VALUE;
	Double prevDistStep = Double.MAX_VALUE;
	Double countDistStep = 0.0;
	FileWriter fw_task;
	BufferedWriter bw_task;
	FileWriter fw_action;
	BufferedWriter bw_action;
	public PrintWriter out_task;
	public PrintWriter out_action;

	@Override
	public void init() {
		logger.setLevel(Level.OFF);

		MessageListener<PointAtStatus> ml_point_at = new MessageListener<PointAtStatus>() {
			public void onNewMessage(PointAtStatus status) {
				try {
					switch (status.getStatus()) {
					case 0:
						getTS().getAg().addBel(Literal.parseLiteral("point_at(idle)"));
						break;
					case 1:
						getTS().getAg().addBel(Literal.parseLiteral("point_at(rotate)"));
						break;
					case 2:
						getTS().getAg().addBel(Literal.parseLiteral("point_at(point)"));
						break;
					case 3:
						getTS().getAg().addBel(Literal.parseLiteral("point_at(finished)"));
						break;
					}
				} catch (RevisionFailedException e) {
					Tools.getStackTrace(e);
				}

			}
		};
		rosnode.addListener("guiding/topics/point_at_status", PointAtStatus._TYPE, ml_point_at);

		MessageListener<LookAtStatus> ml_look_at = new MessageListener<LookAtStatus>()  {
			public void onNewMessage(LookAtStatus status) {
				try {
					switch (status.getStatus()) {
					case 0:
						getTS().getAg().addBel(Literal.parseLiteral("look_at(idle)"));
						break;
					case 1:
						getTS().getAg().addBel(Literal.parseLiteral("look_at(rotate)"));
						break;
					case 2:
						getTS().getAg().addBel(Literal.parseLiteral("look_at(look)"));
						break;
					case 3:
						getTS().getAg().addBel(Literal.parseLiteral("look_at(finished)"));
						break;
					}
				} catch (RevisionFailedException e) {
					Tools.getStackTrace(e);
				}

			}
		};
		rosnode.addListener("guiding/topics/look_at_status", LookAtStatus._TYPE, ml_look_at);
		

		MessageListener<Float64> ml_ttg = new MessageListener<Float64>()  {
			public void onNewMessage(Float64 n) {
				if(firstTtg.equals(Double.MAX_VALUE)) {
					firstTtg = n.getData();
					moveStarted = getCurrentTime();
				}
				ttg = n.getData();

			}
		};
		rosnode.addListener("guiding/topics/ttg", Float64._TYPE, ml_ttg);
		
		
		try {
			fw_task = new FileWriter("log/qoi/task.txt", true);
			fw_action = new FileWriter("log/qoi/action.txt", true);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    bw_task = new BufferedWriter(fw_task);
	    bw_action = new BufferedWriter(fw_action);
	    out_task = new PrintWriter(bw_task);
	    out_action = new PrintWriter(bw_action);
		super.init();
	}
	

	@Override
	public void reasoningCycleStarting() {
		Literal onGoingTask = findBel("started");
		double t = getCurrentTime();
		logger.info("current time :"+t);
		logger.info("prev time :"+prevTime);
		double diff = (t-prevTime)/1000;
		logger.info("diff time :"+diff);
		if(!onGoingStep.isEmpty() && onGoingTask != null && diff >= 0.8) {
			if(firstTimeInTask) {
				display.insert_discontinuity("task", getRosTimeMilliSeconds());
				firstTimeInTask = false;
				startTimeOngoingStep = ((NumberTermImpl)((Literal) onGoingTask.getAnnots("add_time").get(0)).getTerm(0)).solve();
			}
			
			Literal attentive_ratio = null;
			Literal action_expectation = null;
			Literal action_efficiency = null;
			ArrayList<Literal> list = new ArrayList<Literal>();

			String id = onGoingTask.getAnnots().get(0).toString();

			if(actionsQoI.get(id) == null)
				actionsQoI.put(id, new QoIDB());
			if(tasksQoI.get(id) == null)
				tasksQoI.put(id, new QoIDB());

			// attentive ratio
			// inform & questions
			startActionLock.readLock().lock();
			if(startTimeOngoingAction != -1.0) {
				double ar = ((InteractAgArch) interact_arch).attentive_ratio(startTimeOngoingAction, getRosTimeMilliSeconds());
				startActionLock.readLock().unlock();
				attentive_ratio = literal("attentive_ratio",onGoingAction,QoI.normaFormulaMinus1To1(ar, 0, 1));
				logger.info("attentive ratio :"+attentive_ratio.toString());
				actionsQoI.get(id).add(attentive_ratio);
				list.add(attentive_ratio);
			} else {
				startActionLock.readLock().unlock();
			}
			
			// action expectations
			// questions
			if((inQuestion || humanAnswer != 0) && !onGoingAction.equals("")) {
//				double questionDuration = getRosTimeMilliSeconds() - startTimeOngoingAction;
//				if(humanAnswer == 0)
//					monitorTimeAnswering = Math.max(-Math.max(questionDuration - actionsThreshold.get(onGoingAction), 0)
//																/ (actionsThreshold.get(onGoingAction) * decreasingSpeed) + 1 , -1);	
				//TODO corriger: n est jamais reinitialisé
				Literal not_exp_ans = findBel("not_exp_ans(_)");
				double c = 0;
				if(not_exp_ans != null) {
					c = ((NumberTermImpl) not_exp_ans.getTerm(0)).solve()+1;
				}
				action_efficiency = literal("action_efficiency","question",QoI.normaFormulaMinus1To1(c, 3, 0));
				logger.info("question action efficiency :"+action_efficiency.toString());
				actionsQoI.get(id).add(action_efficiency);
				list.add(action_efficiency);
				
//				action_expectation = literal("action_expectation", onGoingAction,monitorTimeAnswering);
//				logger.info("question action expectation :"+action_expectation.toString());
//				actionsQoI.get(id).add(action_expectation);
//				list.add(action_expectation);
			} else {
				
				Literal come_front = findBel("wait_human");
				if(come_front != null) {
					onGoingAction = "wait_human";

					// action efficiency
					Literal said = findBel("said(come,_)");
					double c = 0;
					if(said != null) {
						c = ((NumberTermImpl) said.getTerm(1)).solve();
					}
					action_efficiency = literal("action_efficiency","come_front",QoI.normaFormulaMinus1To1(c, 3, 0));
					actionsQoI.get(id).add(action_efficiency);
					list.add(action_efficiency);
//					logger.info(action_efficiency.toString());
					if(!wasWaiting)
						startAction = true;
					wasWaiting = true;
				} else if(wasWaiting) {
					display.insert_discontinuity("action", getRosTimeMilliSeconds());
					wasWaiting = false;
				}


				// attentive ratio & action expectations & action efficiency
				// come closer
				
				if(humanAnswer == 0){
					Literal closer = findBel("adjust");
					if(closer != null) {
						onGoingAction = "come_closer";
						Literal distToGoal = findBel("dist_to_goal(_,_,_)");
						if(distToGoal != null) {
							ArrayList<Double> goal = Tools.listTermNumbers_to_list((ListTermImpl) distToGoal.getTerm(2));
							Transform human_pose_now = getTfTree().lookupMostRecent("map", distToGoal.getTerm(0).toString().replaceAll("^\"|\"$", ""));
							double distComeCloser = Math.hypot(human_pose_now.translation.x - goal.get(0), 
									human_pose_now.translation.y - goal.get(1));
							logger.info("dist now :"+distComeCloser);
							logger.info("dist prev :"+prevDistComeCloser);
							if((Math.abs(prevDistComeCloser - distComeCloser) > 0.1 && prevDistComeCloser > distComeCloser)  || prevDistComeCloser.equals(Double.MAX_VALUE)) {
								logger.info("pos");
								if(countDistComeCloser > 0)
									countDistComeCloser += -1;
							}else {
								countDistComeCloser += 1;
								logger.info("neg");
							}
							prevDistComeCloser = distComeCloser;
							action_expectation = literal("action_expectation","come_closer",QoI.logaFormulaMinus1To1(countDistComeCloser, 5, 1.5) * (-1));
							actionsQoI.get(id).add(action_expectation);
							list.add(action_expectation);
							
						}

						// action efficiency
						Literal said = findBel("said(closer,_)");
						double c = 0;
						if(said != null) {
							c = ((NumberTermImpl) said.getTerm(1)).solve();
						}
						action_efficiency = literal("action_efficiency","come_closer",QoI.normaFormulaMinus1To1(c, 3, 0)); 
						actionsQoI.get(id).add(action_efficiency);
						list.add(action_efficiency);
						logger.info(action_efficiency.toString());
						if(!wasComingCloser) {
							startAction = true;
							logger.info("start action");
						}
						wasComingCloser = true;
					} 
					else if(wasComingCloser) {
						logger.info("insert discontinuity");
						display.insert_discontinuity("action", getRosTimeMilliSeconds());
						wasComingCloser = false;
					}

					// step on side
					Literal step = findBel("step");
					if(step != null) {
						
						onGoingAction = "step";
						// attentive ratio
//						double startTime = ((NumberTermImpl) step.getAnnot("add_time").getTerm(0)).solve();
//						double ar = ((InteractAgArch) interact_arch).attentive_ratio(startTime, getRosTimeMilliSeconds());
//						attentive_ratio = literal("attentive_ratio","step",QoI.normaFormulaMinus1To1(ar, 0, 1));
//						actionsQoI.get(id).add(attentive_ratio);
//						list.add(attentive_ratio);

						// action expectations
						Literal distToGoal = findBel("dist_to_goal(_,_,_)");
						if(distToGoal != null) {
							ArrayList<Double> goal = Tools.listTermNumbers_to_list((ListTermImpl) distToGoal.getTerm(2));
							Transform human_pose_now = getTfTree().lookupMostRecent("map", distToGoal.getTerm(0).toString().replaceAll("^\"|\"$", ""));
							double distStep = Math.hypot(human_pose_now.translation.x - goal.get(0), 
									human_pose_now.translation.y - goal.get(1));
							logger.info("dist now :"+distStep);
							logger.info("dist prev :"+prevDistStep);
							if((Math.abs(prevDistStep - distStep) > 0.1 && prevDistStep < distStep)  || prevDistStep.equals(Double.MAX_VALUE)) {
								logger.info("pos");
								if(countDistStep > 0)
									countDistStep += -1;
							}else {
								countDistStep += 1;
								logger.info("neg");
							}
							prevDistStep = distStep;
							action_expectation = literal("action_expectation","step",QoI.logaFormulaMinus1To1(countDistStep, 5, 1.5) * (-1));
							actionsQoI.get(id).add(action_expectation);
							list.add(action_expectation);
							
							if(!wasStepping)
								startAction = true;
							wasStepping = true;
						}

						// action efficiency
						Literal said = findBel("said(step(_),_)");
						double c = 0;
						if(said != null) {
							c = ((NumberTermImpl) said.getTerm(1)).solve();
						}
						action_efficiency = literal("action_efficiency","step",QoI.normaFormulaMinus1To1(c, 3, 0));
						actionsQoI.get(id).add(action_efficiency);
						list.add(action_efficiency);
					} 
					else if(wasStepping) {
						display.insert_discontinuity("action", getRosTimeMilliSeconds());
						wasStepping = false;
					}
					
					// robot moves
					Literal move = findBel("move(started)");
					Literal move_over = findBel("move(over)");
					if(move != null && move_over == null) {
						onGoingAction = "robot_move";
						logger.info("move(started) found");
						// action expectations
						
						if(ttg != Double.MAX_VALUE) {
							logger.info("dist to goal :"+ttg);
							double moveDuration = ( getCurrentTime() - moveStarted) / 1000;
							logger.info("move duration :"+moveDuration);
							double deltaTtg = moveDuration + ttg - firstTtg;
							logger.info("first ttg :"+firstTtg);
							logger.info("delta :"+deltaTtg);
							if(deltaTtg < 0)
								deltaTtg = 0;
							action_expectation = literal("action_expectation","robot_move",QoI.logaFormulaMinus1To1(deltaTtg, 5, 1.5) * (-1));
							actionsQoI.get(id).add(action_expectation);
							list.add(action_expectation);
//							logger.info(action_expectation.toString());
							if(!wasMoving)
								startAction = true;
							wasMoving = true;
						}
						
					} else if(wasMoving) {
						display.insert_discontinuity("action", getRosTimeMilliSeconds());
						wasMoving = false;
					}
				}
			}
			double sum = 0;
			for(Literal l : list) {
				sum += ((NumberTermImpl) l.getTerm(1)).solve();
			}
			double actionsQoIAverage = 0;
			Literal la = null;
			if(!list.isEmpty()) {
				logger.info("list size :"+list.size());
				double QoI = sum/list.size();
				la = literal("qoi",onGoingAction, QoI);
				this.actionsQoI.get(id).add(la);
				currentTaskActQoI.add(QoI);
			}
			actionsQoIAverage = currentTaskActQoI.stream().mapToDouble(val -> val).average().orElse(0.0);
			if(!currentTaskActQoI.isEmpty()) {
				tasksQoI.get(id).add(literal("actionsQoI",id,actionsQoIAverage));
				logger.info("actions qoi average :"+actionsQoIAverage);
			}
			// QoI task - distance to goal and task evolution
			
			distToGoal = step/steps;
			
			onTimeTaskExecution = Math.max(-Math.max(getRosTimeMilliSeconds() - startTimeOngoingStep - stepsThreshold.get(onGoingStep), 0)/(stepsThreshold.get(onGoingStep) * decreasingSpeed) + onTimeTaskExecutionPrev , -1);
			
			tasksQoI.get(id).add(literal("taskDtG",id,distToGoal));
			tasksQoI.get(id).add(literal("taskExecutionEvolution",id,onTimeTaskExecution));
//			logger.info("dist to goal :"+distToGoal);
//			logger.info("taskExecutionEvolution :"+onTimeTaskExecution);
//			Literal lt = literal("qoi",id,(2*actionsQoIAverage+distToGoal+2*onTimeTaskExecution)/5.0);
			Literal lt ;
			if(!currentTaskActQoI.isEmpty())
				lt = literal("qoi",id,(3*actionsQoIAverage+onTimeTaskExecution)/4.0);
			else
				lt = literal("qoi",id,onTimeTaskExecution);
			tasksQoI.get(id).add(lt);
				
//			logger.info(lt.toString());
			display.update(null,lt, la );
			if(newStep) {
				newStep = false;
				display.add_label(onGoingStep, lt);
			}
			if(humanAnswer != 0) {
				humanAnswer = 0;
				display.insert_discontinuity("action", getRosTimeMilliSeconds());
			}
			if(startAction && la != null) {
				startAction = false;
				logger.info("start action to false");
				display.add_label(onGoingAction, la);
			}
			
	    	out_task.println(lt.getAnnot("add_time").getTerm(0).toString()+";"+lt.getTerm(1).toString());
	    	if(la != null)
	    		out_action.println(la.getAnnot("add_time").getTerm(0).toString()+";"+la.getTerm(1).toString()+";"+la.getTerm(0).toString());
	    	
//			logger.info(tasksQoI.get(id).toString());
//			logger.info(this.actionsQoI.get(id).toString());
			nbTaskQoI++;
			taskQoIAverage = taskQoIAverage + ( ((NumberTermImpl) lt.getTerm(1)).solve() - taskQoIAverage) / nbTaskQoI;
			Message msg = new Message("tell", getAgName(), "interac", "qoi("+id+","+Double.toString(taskQoIAverage)+")");
			try {
				sendMsg(msg);
			} catch (Exception e) {
				e.printStackTrace();
			}
			prevTime = getCurrentTime();
		}

		
		super.reasoningCycleStarting();
	}
	
	

	@Override
	public void actionExecuted(ActionExec act) {
		String action_name = act.getActionTerm().getFunctor();
		Message msg;
		if (act.getResult()) {
			msg = new Message("tell", getAgName(), "supervisor", "action_over(" + action_name + ")");
		} else {
			if (act.getFailureReason() != null)
				msg = new Message("tell", getAgName(), "supervisor",
						"action_failed(" + action_name + "," + act.getFailureReason().toString() + ")");
			else
				msg = new Message("tell", getAgName(), "supervisor", "action_failed(" + action_name + ")");
		}
		try {
			sendMsg(msg);
		} catch (Exception e) {
			Tools.getStackTrace(e);
		}
		super.actionExecuted(act);
	}

	public void reinit_steps_number() {
		distToGoal = 0;
		steps = 2;
	}

	public void increment_steps_number() {
		steps += 1;
	}

	public void reinit_step() {
		step = 0;
	}

	public void increment_step() {
		step += 1;
		logger.info("achieved step "+this.step+" over "+steps);
		
		
	}
	
	public void set_on_going_step(String step) {
		onGoingStep = step;
		display.setOngoingStep(step);
		logger.info("------------------------------------------ new step : "+step+"-----------------------------");
		newStep = true;
		onTimeTaskExecutionPrev = onTimeTaskExecution;
		startTimeOngoingStep = getRosTimeMilliSeconds();
	}
	
	public double task_achievement() {
		return step/steps;
	}
	
	public void reinit_qoi_variables() {
		display.insert_discontinuity("task", getRosTimeMilliSeconds());
		currentTaskActQoI.clear();
		firstTimeInTask = true;
		onGoingStep = "";
		onGoingAction = "";
		startActionLock.writeLock().lock();
		startTimeOngoingAction = -1;
		startActionLock.writeLock().unlock();
		wasStepping = false;
		wasMoving = false;
		wasComingCloser = false;
		onTimeTaskExecution = 1;
		onTimeTaskExecutionPrev = 1;
		firstTtg = Double.MAX_VALUE;
		ttg = Double.MAX_VALUE;
		prevDistComeCloser = Double.MAX_VALUE;
		countDistComeCloser = 0.0;
		prevDistStep = Double.MAX_VALUE;
		countDistStep = 0.0;
	}
	
	public void startAction(boolean isQuestion) {
		if(isQuestion)
			onGoingAction = "question";
		else
			onGoingAction = "speak";
		startTimeOngoingAction = getRosTimeMilliSeconds();
//		logger.info("start action to true with "+text);
		startAction = true;
	}
	
	public void setStartTimeAction() {
		startTimeOngoingAction = getRosTimeMilliSeconds();
	}
	
	public void endAction() {
		startActionLock.writeLock().lock();
		startTimeOngoingAction = -1;
		startActionLock.writeLock().unlock();
//		logger.info("insert discontinuity");
		display.insert_discontinuity("action", getRosTimeMilliSeconds());
		
	}
	
	public LinkedList<Literal> getTaskQoI(String id) {
		return tasksQoI.get(id);
	}
	
	public LinkedList<Literal> getActionQoI(String id) {
		return actionsQoI.get(id);
	}
	
	public void setInQuestion(boolean b) {
		inQuestion = b;
	}
	
	public void setHumanAnswer(int i) {
		humanAnswer = 0;
	}

	public int getHumanAnswer() {
		return humanAnswer;
	}
	
	public void writeLockStartAction() {
		startActionLock.writeLock().lock();
	}
	
	public void writeUnlockStartAction() {
		startActionLock.writeLock().unlock();
	}
	
	public TransformTree getTfTree() {
		return ((RosNodeGuiding) rosnode).getTfTree();
	}


};
