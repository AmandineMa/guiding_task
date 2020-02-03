package ros;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Stack;
import java.util.UUID;
import java.util.logging.Logger;

import org.ros.exception.ParameterNotFoundException;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.message.Duration;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.TransformTree;
import org.ros.rosjava.tf.pubsub.TransformListener;

import com.github.rosjava_actionlib.ActionServer;
import com.github.rosjava_actionlib.ActionServerListener;
import com.github.rosjava_actionlib.GoalIDGenerator;
import com.google.common.collect.ArrayListMultimap;
import com.google.common.collect.Multimap;
import com.google.common.collect.Multimaps;

import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;
import dialogue_arbiter.DialogueArbiterActionGoal;
import dialogue_arbiter.DialogueArbiterGoal;
import dialogue_as.dialogue_actionActionFeedback;
import dialogue_as.dialogue_actionActionGoal;
import dialogue_as.dialogue_actionActionResult;
import dialogue_as.dialogue_actionGoal;
import geometry_msgs.PoseStamped;
import guiding_as_msgs.taskActionFeedback;
import guiding_as_msgs.taskActionGoal;
import guiding_as_msgs.taskActionResult;
import guiding_as_msgs.taskResult;
import move_base_msgs.MoveBaseActionFeedback;
import move_base_msgs.MoveBaseActionGoal;
import move_base_msgs.MoveBaseActionResult;
import move_base_msgs.MoveBaseGoal;
import pepper_base_manager_msgs.StateMachineStatePrioritizedAngle;
import pepper_resources_synchronizer_msgs.SubStateMachine_pepper_base_manager_msgs;
import perspectives_msgs.Fact;
import perspectives_msgs.FactArrayStamped;
import perspectives_msgs.GetNameResponse;
import pointing_planner.PointingActionFeedback;
import pointing_planner.PointingActionResult;
import resource_management_msgs.EndCondition;
import resource_management_msgs.MessagePriority;
import resource_management_msgs.StateMachineStateHeader;
import resource_management_msgs.StateMachineTransition;
import resource_synchronizer_msgs.MetaStateMachineHeader;
import resource_synchronizer_msgs.SubStateMachineHeader;
import rjs.ros.AbstractRosNode;
import rjs.utils.SimpleFact;
import rjs.utils.Tools;
import rpn_recipe_planner_msgs.SupervisionServerInformActionGoal;
import rpn_recipe_planner_msgs.SupervisionServerInformActionResult;
import rpn_recipe_planner_msgs.SupervisionServerInformGoal;
import rpn_recipe_planner_msgs.SupervisionServerQueryActionGoal;
import rpn_recipe_planner_msgs.SupervisionServerQueryActionResult;
import rpn_recipe_planner_msgs.SupervisionServerQueryGoal;
import std_srvs.EmptyResponse;

/***
 * ROS node to be used by Jason
 * 
 * @author Google Code
 * @version 1.0
 * @since 2014-06-02
 *
 */
public class RosNodeGuiding extends AbstractRosNode {
	private Logger logger = Logger.getLogger(RosNodeGuiding.class.getName());

	private ActionServer<taskActionGoal, taskActionFeedback, taskActionResult> guidingAS;
	private taskActionGoal newGuidingGoal = null;
	private Stack<taskActionGoal> stackGuidingGoals;
	private Subscriber<perspectives_msgs.FactArrayStamped> factsSub;
	private PointingActionResult placementsResult;
	private PointingActionFeedback placementsFb;
	private dialogue_actionActionResult listeningResult;
	private dialogue_actionActionFeedback listeningFb;
	private SupervisionServerInformActionResult listeningResultInform;
	private SupervisionServerQueryActionResult listeningResultQuery;
	private MoveBaseActionResult moveToResult;
	private MoveBaseActionFeedback moveToFb;
	private Publisher<MoveBaseActionGoal> moveToGoalPub;
	private Publisher<dialogue_actionActionGoal> dialoguePub;
	private Publisher<SupervisionServerInformActionGoal> dialoguePubInform;
	private GoalID informGoalId;
	private Publisher<GoalID> dialogueCancelPubInform;
	private Publisher<SupervisionServerQueryActionGoal> dialoguePubQuery;
	private GoalID queryGoalId;
	private Publisher<GoalID> dialogueCancelPubQuery;
	private Publisher<GoalID> dialogueCancelPub;
	private Publisher<DialogueArbiterActionGoal> engagePub;
	private Publisher<visualization_msgs.Marker> markerPub;
	private Publisher<std_msgs.Int32> personOfInterestPub;
	private Multimap<String, SimpleFact> perceptions = Multimaps
			.synchronizedMultimap(ArrayListMultimap.<String, SimpleFact>create());
	private volatile int perceptID = 0;
	private TransformListener tfl;
	private GoalIDGenerator goalIDGenerator;

	public RosNodeGuiding(String name) {
		super(name);
	}

	public GraphName getDefaultNodeName() {
		return GraphName.of("supervisor_clients");
	}
	

	@Override
	public void onShutdown(Node node) {
		cancelDialogueInformGoal();
		cancelDialogueQueryGoal();
		
		// code degueu, ne devrait pas être là
		ServiceResponseListener<std_srvs.EmptyResponse> respListener = new ServiceResponseListener<std_srvs.EmptyResponse>() {

			@Override
			public void onFailure(RemoteException e) {}

			@Override
			public void onSuccess(EmptyResponse arg0) {}
		};
		callAsyncService("terminate_interaction", respListener, null);
		super.onShutdown(node);
	}

	@SuppressWarnings("unchecked")
	public void init() {
		try {
//			goalIDGenerator = new GoalIDGenerator(getConnectedNode());
//			tfl = new TransformListener(connectedNode);
//			parameters = connectedNode.getParameterTree();
//			URI uri = null;
//			try {
//				uri = new URI(System.getenv("ROS_MASTER_URI"));
//			} catch (URISyntaxException e) {
//				logger.info("Wrong URI syntax :" + e.getMessage());
//			}
//			msc = new MasterStateClient(connectedNode, uri);
//			serviceClients = new HashMap<String, ServiceClient<Message, Message>>();
//			servicesMap = null;
			super.init();
			
			goalIDGenerator = new GoalIDGenerator(getConnectedNode());
			tfl = new TransformListener(connectedNode);
			
			if(parameters.has("/guiding/base_services"))
				servicesMap = (HashMap<String, HashMap<String, String>>) parameters.getMap("/guiding/base_services");
			
			if(parameters.has("/guiding/services_moving") && !parameters.getBoolean("/guiding/immo")) {
				HashMap<String, HashMap<String, String>> map = (HashMap<String, HashMap<String, String>>) parameters.getMap("/guiding/services_moving");
				servicesMap.putAll(map);
			}
			
			if(parameters.has("/guiding/services_hwu") && parameters.getBoolean("/guiding/dialogue/hwu")) {
				HashMap<String, HashMap<String, String>> map = (HashMap<String, HashMap<String, String>>) parameters.getMap("/guiding/services_hwu");
				servicesMap.putAll(map);
			}
			
			if(parameters.has("/guiding/services_wo_hwu") && !parameters.getBoolean("/guiding/dialogue/hwu")) {
				HashMap<String, HashMap<String, String>> map = (HashMap<String, HashMap<String, String>>) parameters.getMap("/guiding/services_wo_hwu");
				servicesMap.putAll(map);
			}
			
			stackGuidingGoals = new Stack<taskActionGoal>();
			markerPub = getConnectedNode().newPublisher("/pp_debug", visualization_msgs.Marker._TYPE);
			personOfInterestPub = getConnectedNode().newPublisher(parameters.getString("/guiding/topics/person_of_interest"), std_msgs.Int32._TYPE);
			guidingAS = new ActionServer<>(connectedNode, "/guiding_task", taskActionGoal._TYPE, taskActionFeedback._TYPE, taskActionResult._TYPE);

			if(parameters.has("/guiding/action_servers/move_to")) {
				moveToGoalPub = connectedNode.newPublisher(
						parameters.getString("/guiding/action_servers/move_to") + "/goal", MoveBaseActionGoal._TYPE);
				
				MessageListener<MoveBaseActionResult> ml_move = new MessageListener<MoveBaseActionResult>() {
	
					@Override
					public void onNewMessage(MoveBaseActionResult result) {
						moveToResult = result;
					}
				};
				addListenerResult("/guiding/action_servers/move_to", MoveBaseActionResult._TYPE, ml_move);
			}
			
			if(parameters.has("/guiding/action_servers/dialogue_inform") && parameters.has("/guiding/action_servers/dialogue_query") 
					&& parameters.getBoolean("guiding/dialogue/hwu")) {
				dialoguePubInform = connectedNode.newPublisher(
						parameters.getString("/guiding/action_servers/dialogue_inform") + "/goal", SupervisionServerInformActionGoal._TYPE);
				
				dialoguePubQuery = connectedNode.newPublisher(
						parameters.getString("/guiding/action_servers/dialogue_query") + "/goal", SupervisionServerQueryActionGoal._TYPE);
				
				MessageListener<SupervisionServerInformActionResult> ml_inform = new MessageListener<SupervisionServerInformActionResult>() {
					
					@Override
					public void onNewMessage(SupervisionServerInformActionResult result) {
						listeningResultInform = result;
					}
				};
				addListenerResult("/guiding/action_servers/dialogue_inform", SupervisionServerInformActionResult._TYPE, ml_inform);
				dialogueCancelPubInform = connectedNode.newPublisher(
						parameters.getString("/guiding/action_servers/dialogue_inform") + "/cancel", GoalID._TYPE);
				
				MessageListener<SupervisionServerQueryActionResult> ml_query = new MessageListener<SupervisionServerQueryActionResult>() {
					
					@Override
					public void onNewMessage(SupervisionServerQueryActionResult result) {
						listeningResultQuery = result;
					}
				};
				addListenerResult("/guiding/action_servers/dialogue_query", SupervisionServerQueryActionResult._TYPE, ml_query);
				dialogueCancelPubQuery = connectedNode.newPublisher(
						parameters.getString("/guiding/action_servers/dialogue_query") + "/cancel", GoalID._TYPE);
				
			}
			
			if(parameters.has("/guiding/action_servers/dialogue") && !parameters.getBoolean("guiding/dialogue/hwu")) {
				dialoguePub = connectedNode.newPublisher(
						parameters.getString("/guiding/action_servers/dialogue") + "/goal", dialogue_actionActionGoal._TYPE);
				dialogueCancelPub = connectedNode.newPublisher(
						parameters.getString("/guiding/action_servers/dialogue") + "/cancel", GoalID._TYPE);
				
				MessageListener<dialogue_actionActionResult> ml_dialogue = new MessageListener<dialogue_actionActionResult>() {
	
					@Override
					public void onNewMessage(dialogue_actionActionResult result) {
						listeningResult = result;
						if(result.getStatus().getStatus()==actionlib_msgs.GoalStatus.SUCCEEDED) {
							logger.info("result succeeded :"+result.getResult().getSubject());
						}	
					}
				};
				addListenerResult("/guiding/action_servers/dialogue", dialogue_actionActionResult._TYPE, ml_dialogue);
				MessageListener<dialogue_actionActionFeedback> ml_dialogue_fb = new MessageListener<dialogue_actionActionFeedback>() {
					@Override
					public void onNewMessage(dialogue_actionActionFeedback fb) {
						listeningFb = fb;
					}
				};
				addListenerFb("/guiding/action_servers/dialogue", dialogue_actionActionFeedback._TYPE, ml_dialogue_fb);
			}
			
			if(parameters.has("/guiding/action_servers/engage")) {
				engagePub = connectedNode.newPublisher(
						parameters.getString("/guiding/action_servers/engage") + "/goal", DialogueArbiterActionGoal._TYPE);
			}
			

			factsSub = connectedNode.newSubscriber(parameters.getString("/guiding/topics/current_facts"),
					perspectives_msgs.FactArrayStamped._TYPE);

			factsSub.addMessageListener(new MessageListener<perspectives_msgs.FactArrayStamped>() {

				public void onNewMessage(FactArrayStamped facts) {
					synchronized (perceptions) {
						perceptions.clear();
						for (Fact fact : facts.getFacts()) {
							final SimpleFact simple_fact;
							String predicate = fact.getPredicate();
							String subject = fact.getSubjectName();
							String object = fact.getObjectName();
							if (predicate.equals("isVisibleBy")) {
								predicate = "canSee";
								if (subject.startsWith("\""))
									subject = subject.replaceAll("^\"|\"$", "");
								if (serviceClients.get("get_uwds_name") != null) {
									Map<String, Object> parameters = new HashMap<String, Object>();
									parameters.put("id", subject);
									parameters.put("world", "robot/merged_visibilities");
									GetNameResponse uwds_name_resp = callSyncService("get_uwds_name", parameters);
									if(uwds_name_resp != null) {
										subject = uwds_name_resp.getName();
										if (!subject.startsWith("\""))
											subject = "\"" + subject + "\"";
										simple_fact = new SimpleFact(predicate, subject);
										perceptions.put(object, simple_fact);
									}
								}

							} else {
								if (!object.isEmpty()) {
									if (!object.startsWith("\""))
										object = "\"" + object + "\"";
									simple_fact = new SimpleFact(predicate, object);
								} else {
									simple_fact = new SimpleFact(predicate);
								}
								if (!subject.startsWith("\""))
									subject = "\"" + subject + "\"";			
								perceptions.put(subject, simple_fact);
							}
						}
					}
					perceptID = facts.getHeader().getSeq();
				}
			}, 10);

		} catch (ParameterNotFoundException e) {
			logger.severe("Parameter not found exception : " + e.getMessage());
			throw new RosRuntimeException(e);
		}
	}

	public void setTaskResult(String success, String id) {
		if (guidingAS != null) {
			taskActionResult result = messageFactory.newFromType(taskActionResult._TYPE);
			taskResult r = messageFactory.newFromType(taskResult._TYPE);
			GoalID g_id = messageFactory.newFromType(GoalID._TYPE);
			g_id.setId(id);
			GoalStatus status = messageFactory.newFromType(GoalStatus._TYPE);
			status.setGoalId(g_id);
			if (success.equals("succeeded")) {
				guidingAS.setSucceed(id);
				status.setStatus(GoalStatus.SUCCEEDED);
				r.setSuccess(true);
			}else if(success.equals("preempted")) {
				guidingAS.setPreempt(id);
				status.setStatus(GoalStatus.PREEMPTED);
				r.setSuccess(true);
			}else {
				guidingAS.setAbort(id);
				status.setStatus(GoalStatus.ABORTED);
				r.setSuccess(false);
			}
			result.setResult(r);
			result.setStatus(status);
			logger.info("set action server result :"+success);
			guidingAS.sendResult(result);
		} else {
			logger.info("guiding as null");
		}
	}

	public MetaStateMachineHeader buildMetaHeader() {
		nodeConfiguration = NodeConfiguration.newPrivate();
		messageFactory = nodeConfiguration.getTopicMessageFactory();

		MessagePriority priority = messageFactory.newFromType(MessagePriority._TYPE);
		priority.setValue(MessagePriority.URGENT);

		MetaStateMachineHeader metaheader = messageFactory.newFromType(MetaStateMachineHeader._TYPE);
		metaheader.setBeginDeadLine(connectedNode.getCurrentTime().add(new Duration(5)));
		metaheader.setTimeout(new Duration(-1));
		metaheader.setPriority(priority);

		return metaheader;
	}

	public SubStateMachine_pepper_base_manager_msgs buildStateMachinePepperBaseManager(String id, float d) {
		nodeConfiguration = NodeConfiguration.newPrivate();
		messageFactory = nodeConfiguration.getTopicMessageFactory();

		// Build End Condition: __done__
		EndCondition end_condition = messageFactory.newFromType(EndCondition._TYPE);
		end_condition.setDuration(new Duration(-1));
		end_condition.setTimeout(new Duration(-1));
		ArrayList<String> a = new ArrayList<String>();
		a.add("__done__");
		end_condition.setRegexEndCondition(a);

		// Set State Machine Transition from End Condition
		StateMachineTransition smtransition = messageFactory.newFromType(StateMachineTransition._TYPE);
		smtransition.setEndCondition(end_condition);
		smtransition.setNextState("end");

		// Set State Header from Transition
		StateMachineStateHeader stateheader = messageFactory.newFromType(StateMachineStateHeader._TYPE);
		stateheader.setId(id);
		stateheader.setTransitions(Arrays.asList(smtransition));

		// Creating corresponding state
		StateMachineStatePrioritizedAngle state = messageFactory.newFromType(StateMachineStatePrioritizedAngle._TYPE);
		state.setData(d);
		state.setHeader(stateheader);

		pepper_base_manager_msgs.StateMachine statemachine = messageFactory
				.newFromType(pepper_base_manager_msgs.StateMachine._TYPE);
		statemachine.setStatesPrioritizedAngle(Arrays.asList(state));

		SubStateMachineHeader subsmheader = messageFactory.newFromType(SubStateMachineHeader._TYPE);
		subsmheader.setInitialState(id);
		subsmheader.setBeginDeadLine(connectedNode.getCurrentTime().add(new Duration(5)));
		subsmheader.setTimeout(new Duration(-1));

		SubStateMachine_pepper_base_manager_msgs substatemachine = messageFactory
				.newFromType(SubStateMachine_pepper_base_manager_msgs._TYPE);
		substatemachine.setHeader(subsmheader);
		substatemachine.setStateMachine(statemachine);
		return substatemachine;
	}

	public void callMoveToAS(PoseStamped pose) {
		moveToFb = null;
		moveToResult = null;
		MoveBaseActionGoal goal_msg;
		goal_msg = moveToGoalPub.newMessage();

		nodeConfiguration = NodeConfiguration.newPrivate();
		messageFactory = nodeConfiguration.getTopicMessageFactory();
		MoveBaseGoal move_to_goal = messageFactory.newFromType(MoveBaseGoal._TYPE);

		move_to_goal.setTargetPose(pose);
		goal_msg.setGoal(move_to_goal);

		moveToGoalPub.publish(goal_msg);
	}
	
	public void callEngageAS(String human_id) {
		DialogueArbiterActionGoal goal_msg;
		goal_msg = engagePub.newMessage();
		
		DialogueArbiterGoal engage_goal = messageFactory.newFromType(DialogueArbiterGoal._TYPE);
		
		engage_goal.setId(UUID.randomUUID().toString());
		engage_goal.setParams("{\"person_frame\": \"human_"+human_id+"\"}");
		goal_msg.setGoal(engage_goal);
		
		engagePub.publish(goal_msg);
	}
	
	public void callDialogueInformAS(String status, String return_value) {
		listeningResultInform = null;
		SupervisionServerInformActionGoal listen_goal_msg = messageFactory.newFromType(SupervisionServerInformActionGoal._TYPE);
		SupervisionServerInformGoal listen_goal = listen_goal_msg.getGoal();
		listen_goal.setStatus(status);
		if(return_value != null && !return_value.isEmpty())
			listen_goal.setReturnValue(return_value);
		listen_goal_msg.setGoal(listen_goal);
		GoalID goalID = messageFactory.newFromType(GoalID._TYPE);
		goalIDGenerator.generateID(goalID);
		informGoalId = goalID;
		logger.info("inform goal ID: "+informGoalId);
		listen_goal_msg.setGoalId(goalID);
		dialoguePubInform.publish(listen_goal_msg);
	}
	
	public void cancelDialogueInformGoal() {
		if(informGoalId != null)
			dialogueCancelPubInform.publish(informGoalId);
	}
	
	public void cancelDialogueQueryGoal() {
		if(queryGoalId != null)
			dialogueCancelPubQuery.publish(queryGoalId);
	}
	
	public void callDialogueQueryAS(String status, String return_value) {
		listeningResultQuery = null;
		SupervisionServerQueryActionGoal listen_goal_msg = messageFactory.newFromType(SupervisionServerQueryActionGoal._TYPE);
		SupervisionServerQueryGoal listen_goal = listen_goal_msg.getGoal();
		listen_goal.setStatus(status);
		if(return_value != null && !return_value.isEmpty())
			listen_goal.setReturnValue(return_value);
		listen_goal_msg.setGoal(listen_goal);
		GoalID goalID = messageFactory.newFromType(GoalID._TYPE);
		goalIDGenerator.generateID(goalID);
		queryGoalId = goalID;
		logger.info("query goal ID: "+queryGoalId);
		listen_goal_msg.setGoalId(goalID);
		dialoguePubQuery.publish(listen_goal_msg);
	}

	public void callDialogueAS(List<String> subjects) {
		callDialogueAS(subjects, new ArrayList<String>());
	}
	
	public void callDialogueAS(List<String> subjects, List<String> verbs) {
		listeningFb = null;
		listeningResult = null;
		dialogue_actionActionGoal listen_goal_msg = messageFactory.newFromType(dialogue_actionActionGoal._TYPE);
		dialogue_actionGoal listen_goal = listen_goal_msg.getGoal();
		if (!subjects.isEmpty()) {
			listen_goal.setSubjects(subjects);
		} else {
			listen_goal.setEnableOnlyVerb(true);
		}
		if (!verbs.isEmpty()) {
			listen_goal.setVerbs(verbs);
		} else {
			listen_goal.setEnableOnlySubject(true);
		}
		listen_goal_msg.setGoal(listen_goal);
		GoalID goalID = messageFactory.newFromType(GoalID._TYPE);
		goalID.setId("");
		dialogueCancelPub.publish(goalID);
		Tools.sleep(200);
		dialoguePub.publish(listen_goal_msg);
		logger.info("dialogue_as listening");
	}
	
	public void setGuidingASListener(ActionServerListener<taskActionGoal> listener) {
		guidingAS.attachListener(listener);
	}

	public taskActionGoal getNewGuidingGoal() {
		return newGuidingGoal;
	}

	public void setNewGuidingGoal(taskActionGoal current_guiding_goal) {
		this.newGuidingGoal = current_guiding_goal;
	}

	public Stack<taskActionGoal> getStackGuidingGoals() {
		return stackGuidingGoals;
	}

	public PointingActionResult getPlacementsResult() {
		return placementsResult;
	}

	public PointingActionFeedback getPlacementsFb() {
		return placementsFb;
	}

	public SupervisionServerInformActionResult getListeningResultInform() {
		return listeningResultInform;
	}

	public SupervisionServerQueryActionResult getListeningResultQuery() {
		return listeningResultQuery;
	}

	public dialogue_actionActionResult getListeningResult() {
		return listeningResult;
	}

	public dialogue_actionActionFeedback getListeningFb() {
		return listeningFb;
	}

	public MoveBaseActionResult getMoveToResult() {
		return moveToResult;
	}

	public MoveBaseActionFeedback getMoveToFb() {
		return moveToFb;
	}

	public Multimap<String, SimpleFact> getPerceptions() {
		return perceptions;
	}

	public int getPercept_id() {
		return perceptID;
	}

	public void setPerceptions(Multimap<String, SimpleFact> perceptions) {
		this.perceptions = perceptions;
	}

	public ConnectedNode getConnectedNode() {
		return connectedNode;
	}

	public TransformTree getTfTree() {
		return tfl.getTree();
	}
	
	public ParameterTree getParameters() {
		return parameters;
	}

	public Publisher<std_msgs.Int32> getPersonOfInterestPub() {
		return personOfInterestPub;
	}

	public Publisher<visualization_msgs.Marker> getMarkerPub() {
		return markerPub;
	}

	
}