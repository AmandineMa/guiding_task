package arch.agarch;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
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

public class RobotAgArch extends AgArchGuiding {

	protected Logger logger = Logger.getLogger(RobotAgArch.class.getName());
	
	std_msgs.Header her;
	AgArch interact_arch = new InteractAgArch();
	double startTimeOngoingAction = -1;
	double startTimeOngoingStep = -1;
	HashMap<String,Double> actionsThreshold = new HashMap<String,Double>() {{
        put("speak", 10000.0);
        put("question", 15000.0);
        put("robot_move", 20000.0);
        put("come_closer", 30000.0);
        put("step", 20000.0);
    }};
	HashMap<String,Double> stepsThreshold = new HashMap<String,Double>() {{
        put("goal_nego", 30000.0);
        put("person_abilities", 15000.0);
        put("agents_at_right_place", 30000.0);
        put("target_explanation", 20000.0);
        put("direction_explanation", 35000.0);
        put("landmark_seen", 20000.0);
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
	double decreasingSpeed = 2;
	float step = 0;
	ArrayList<Double> currentTaskActQoI = new ArrayList<Double>();
	private final ReadWriteLock startActionLock = new ReentrantReadWriteLock();
	
	double steps = 2;
	double taskQoIAverage = 0;
	double nbTaskQoI = 0;
	boolean firstTimeInTask = true;
	double monitorTimeAnswering = 0;
	boolean wasComingCloser = false;
	boolean wasStepping = false;
	boolean wasMoving = false;
	boolean wasWaiting = false;
	boolean newStep = false;
	boolean startAction = false;

	@Override
	public void init() {

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
		
		super.init();
	}

	@Override
	public void reasoningCycleStarting() {
		Literal onGoingTask = findBel("started");
		if(!onGoingStep.isEmpty() && onGoingTask != null) {
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
//				logger.info(attentive_ratio.toString());
				actionsQoI.get(id).add(attentive_ratio);
				list.add(attentive_ratio);
			} else {
				startActionLock.readLock().unlock();
			}
			
			// action expectations
			// questions
			if((inQuestion || humanAnswer != 0) && !onGoingAction.equals("")) {
				if(humanAnswer == 0)
					monitorTimeAnswering = Math.max(-Math.max(getRosTimeMilliSeconds() - startTimeOngoingAction - actionsThreshold.get(onGoingAction), 0)
																/ (actionsThreshold.get(onGoingAction) * decreasingSpeed) + 1 , -1);	
//				else if(attentive_ratio == null)  {
//					attentive_ratio = findBel(Literal.parseLiteral("attentive_ratio("+onGoingAction+",_)"), this.actionsQoI.get(id));
//					actionsQoI.get(id).add(attentive_ratio);
//					list.add(attentive_ratio);
//				}
					
				action_expectation = literal("action_expectation", onGoingAction,monitorTimeAnswering);
//				logger.info(action_expectation.toString());
				actionsQoI.get(id).add(action_expectation);
				list.add(action_expectation);
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
					action_efficiency = literal("action_efficiency","come_front",QoI.logaFormula(c, 2, 2) * (-1));
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
						// attentive ratio
						double startTime = ((NumberTermImpl) closer.getAnnot("add_time").getTerm(0)).solve();
						double ar = ((InteractAgArch) interact_arch).attentive_ratio(startTime, getRosTimeMilliSeconds());
						attentive_ratio = literal("attentive_ratio","come_closer",QoI.normaFormulaMinus1To1(ar, 0, 1));
						list.add(attentive_ratio);
						actionsQoI.get(id).add(attentive_ratio);
//						logger.info(attentive_ratio.toString());
						// action expectations
						Literal distToGoal = findBel("dist_to_goal(_,_,_)");
						if(distToGoal != null) {
//							logger.info(distToGoal.toString());
							Transform human_pose_now = getTfTree().lookupMostRecent("map", distToGoal.getTerm(0).toString().replaceAll("^\"|\"$", ""));
							ArrayList<Double> init_pose = Tools.listTermNumbers_to_list((ListTermImpl) distToGoal.getTerm(1));
//							logger.info("init pose :"+init_pose);
							if(human_pose_now != null) {
								double h_dist_to_init_pose = Math.hypot(human_pose_now.translation.x - init_pose.get(0), 
										human_pose_now.translation.y - init_pose.get(1));
//								logger.info("dist from initial pose :"+h_dist_to_init_pose);
								ArrayList<Double> goal = Tools.listTermNumbers_to_list((ListTermImpl) distToGoal.getTerm(2));
								double init_dist_to_goal = Math.hypot(init_pose.get(0) - goal.get(0), 
										init_pose.get(1) - goal.get(1));
//								logger.info("initial dist to goal "+init_dist_to_goal);
								double h_dist_to_goal = Math.hypot(human_pose_now.translation.x - goal.get(0), 
										human_pose_now.translation.y - goal.get(1));
//								logger.info("dist to goal "+h_dist_to_goal);
								double ratio = h_dist_to_init_pose/init_dist_to_goal;
								if(h_dist_to_goal > init_dist_to_goal)
									ratio = -1 * ratio;
								action_expectation = literal("action_expectation","come_closer",ratio);
								actionsQoI.get(id).add(action_expectation);
								list.add(action_expectation);
//								logger.info(action_expectation.toString());
							}
						}

						// action efficiency
						Literal said = findBel("said(closer,_)");
						double c = 0;
						if(said != null) {
							c = ((NumberTermImpl) said.getTerm(1)).solve();
						}
						action_efficiency = literal("action_efficiency","come_closer",QoI.logaFormula(c, 2, 2) * (-1));
						actionsQoI.get(id).add(action_efficiency);
						list.add(action_efficiency);
//						logger.info(action_efficiency.toString());
						if(!wasComingCloser)
							startAction = true;
						wasComingCloser = true;
					} else if(wasComingCloser) {
						display.insert_discontinuity("action", getRosTimeMilliSeconds());
						wasComingCloser = false;
					}

					// step on side
					Literal step = findBel("step");
					if(step != null) {
						
						onGoingAction = "step";
						// attentive ratio
						double startTime = ((NumberTermImpl) step.getAnnot("add_time").getTerm(0)).solve();
						double ar = ((InteractAgArch) interact_arch).attentive_ratio(startTime, getRosTimeMilliSeconds());
						attentive_ratio = literal("attentive_ratio","step",QoI.normaFormulaMinus1To1(ar, 0, 1));
						actionsQoI.get(id).add(attentive_ratio);
						list.add(attentive_ratio);

						// action expectations
						Literal distToGoal = findBel("dist_to_goal(_,_,_)");
						if(distToGoal != null) {
							Transform human_pose_now = getTfTree().lookupMostRecent("map", distToGoal.getTerm(0).toString().replaceAll("^\"|\"$", ""));
							ArrayList<Double> init_pose = Tools.listTermNumbers_to_list((ListTermImpl) distToGoal.getTerm(1));
							ArrayList<Double> robot_place = Tools.listTermNumbers_to_list((ListTermImpl) distToGoal.getTerm(2));

							double h_init_pose_dist_to_robot_place = Math.hypot(robot_place.get(0) - init_pose.get(0), 
									robot_place.get(1) - init_pose.get(1));

							double dist_threshold = rosnode.getParameters().getDouble("guiding/tuning_param/human_move_first_dist_th");
							double h_dist_to_robot_place = Math.hypot(human_pose_now.translation.x - robot_place.get(0), 
									human_pose_now.translation.y - robot_place.get(1));
							double value;
							if(h_dist_to_robot_place > h_init_pose_dist_to_robot_place) {
								value = QoI.normaFormul0To1(h_dist_to_robot_place, h_init_pose_dist_to_robot_place, dist_threshold);
							} else {
								value = QoI.normaFormulaMinus1To0(h_dist_to_robot_place, 0, h_init_pose_dist_to_robot_place);
							}
							action_expectation = literal("action_expectation","step",value);
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
						action_efficiency = literal("action_efficiency","step",QoI.logaFormula(c, 2, 2) * (-1));
						actionsQoI.get(id).add(action_efficiency);
						list.add(action_efficiency);
					}  else if(wasStepping) {
						display.insert_discontinuity("action", getRosTimeMilliSeconds());
						wasStepping = false;
					}
					
					// robot moves
					Literal move = findBel("move(started)");
					Literal move_over = findBel("move(over)");
					if(move != null && move_over == null) {
						onGoingAction = "robot_move";
//						logger.info("move(started) found");
						// action expectations
						Literal distToGoal = findBel("dist_to_goal(_,_,_)");
						if(distToGoal != null) {
//							logger.info("dist_to_goal found");
							Literal robot_move_l = findBel("robot_move(_,_,_)");
							ArrayList<Double> robot_pose = null;
							if(robot_move_l != null) {
//								logger.info("robot_move found");
								robot_pose = Tools.listTermNumbers_to_list((ListTermImpl) robot_move_l.getTerm(1));
							}
							if(robot_pose != null) {
								TransformTree tfTree = getTfTree();
								Transform robot_pose_now;
								robot_pose_now = tfTree.lookupMostRecent("map", "base_footprint");
								if(robot_pose_now != null) {
									
									ArrayList<Double> init_pose = Tools.listTermNumbers_to_list((ListTermImpl) distToGoal.getTerm(1));

									double r_dist_to_init_pose = Math.hypot(robot_pose_now.translation.x - init_pose.get(0), 
											robot_pose_now.translation.y - init_pose.get(1));

									ArrayList<Double> goal = Tools.listTermNumbers_to_list((ListTermImpl) distToGoal.getTerm(2));
									double init_dist_to_goal = Math.hypot(init_pose.get(0) - goal.get(0), 
											init_pose.get(1) - goal.get(1));
									double r_dist_to_goal = Math.hypot(robot_pose_now.translation.x - goal.get(0), 
											robot_pose_now.translation.y - goal.get(1));
									double ratio = r_dist_to_init_pose/init_dist_to_goal;
									if(r_dist_to_goal > init_dist_to_goal)
										ratio = -1 * ratio;
									action_expectation = literal("action_expectation","robot_move",ratio);
									actionsQoI.get(id).add(action_expectation);
									list.add(action_expectation);
//									logger.info(action_expectation.toString());
									if(!wasMoving)
										startAction = true;
									wasMoving = true;
								}
							}
							
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
				double QoI = sum/list.size();
				la = literal("qoi",onGoingAction, QoI);
				this.actionsQoI.get(id).add(la);
				currentTaskActQoI.add(QoI);
			}
			actionsQoIAverage = currentTaskActQoI.stream().mapToDouble(val -> val).average().orElse(0.0);
			if(!currentTaskActQoI.isEmpty()) {
				tasksQoI.get(id).add(literal("actionsQoI",id,actionsQoIAverage));
//				logger.info("actions qoi average :"+actionsQoIAverage);
			}
			// QoI task - distance to goal and task evolution
			
			distToGoal = step/steps;
			
			onTimeTaskExecution = Math.max(-Math.max(getRosTimeMilliSeconds() - startTimeOngoingStep - stepsThreshold.get(onGoingStep), 0)/(stepsThreshold.get(onGoingStep) * decreasingSpeed) + onTimeTaskExecutionPrev , -1);
			
			tasksQoI.get(id).add(literal("taskDtG",id,distToGoal));
			tasksQoI.get(id).add(literal("taskExecutionEvolution",id,onTimeTaskExecution));
//			logger.info("dist to goal :"+distToGoal);
//			logger.info("taskExecutionEvolution :"+onTimeTaskExecution);
			Literal lt = literal("qoi",id,(actionsQoIAverage+distToGoal+onTimeTaskExecution)/3.0);
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
//				logger.info("start action to false");
				display.add_label(onGoingAction, la);
			}
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
	}
	
	public void set_on_going_step(String step) {
		onGoingStep = step;
		display.setOngoingStep(step);
//		logger.info("------------------------------------------"+step+"-----------------------------");
//		logger.info("step "+this.step+" over "+steps);
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
