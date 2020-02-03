package arch.actions;

import org.ros.node.topic.Publisher;

import arch.actions.internal.CanBeVisible;
import arch.actions.internal.ComputeRoute;
import arch.actions.internal.GetOntoIndividualInfo;
import arch.actions.internal.GetPlacements;
import arch.actions.internal.GetRouteVerba;
import arch.actions.internal.HasMesh;
import arch.actions.internal.SetParam;
import arch.actions.robot.EnableAnimatedSpeech;
import arch.actions.robot.Engage;
import arch.actions.robot.FaceHuman;
import arch.actions.robot.HumanToMonitor;
import arch.actions.robot.Listen;
import arch.actions.robot.Localise;
import arch.actions.robot.LookAt;
import arch.actions.robot.MoveTo;
import arch.actions.robot.PauseASR;
import arch.actions.robot.PointAt;
import arch.actions.robot.PubLookAtEvents;
import arch.actions.robot.ReinitLoca;
import arch.actions.robot.Rotate;
import arch.actions.robot.TerminateInteraction;
import arch.actions.robot.TextToSpeech;
import arch.actions.ros.InitGuidingAs;
import arch.actions.ros.SetGuidingResult;
import arch.actions.ros.StartROSNode;
import jason.asSemantics.ActionExec;
import rjs.arch.actions.AbstractActionFactory;
import rjs.arch.actions.Action;
import rjs.arch.actions.ros.ConfigureNode;
import rjs.arch.actions.ros.InitServices;
import rjs.arch.actions.ros.RetryInitServices;
import rjs.arch.actions.ros.StartParameterLoaderNode;
import rjs.arch.agarch.AbstractROSAgArch;

public class ActionFactoryImpl extends AbstractActionFactory {
	
	private static Publisher<std_msgs.String> lookAtEventsPub; 
	private static Publisher<std_msgs.String> humanToMonitorPub; 
	
	public void setRosVariables() {
		super.setRosVariables();
		lookAtEventsPub = createPublisher("guiding/topics/look_at_events");
		humanToMonitorPub = createPublisher("guiding/topics/human_to_monitor");
	}
			
	public Action createAction(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		String actionName = actionExec.getActionTerm().getFunctor();
		Action action = null;
		
		switch(actionName) {
			case "human_to_monitor" :
				action = new HumanToMonitor(actionExec, rosAgArch, humanToMonitorPub);
				break;
			case "compute_route" :
				action = new ComputeRoute(actionExec, rosAgArch);
				break;
			case "pause_asr_and_display_processing" :
				action = new PauseASR(actionExec, rosAgArch);
				break;
			case "get_onto_individual_info" :
				action = new GetOntoIndividualInfo(actionExec, rosAgArch);
				break;
			case "get_placements" :
				action = new GetPlacements(actionExec, rosAgArch);
				break;
			case "has_mesh" :
				action = new HasMesh(actionExec, rosAgArch);
				break;
			case "can_be_visible" :
				action = new CanBeVisible(actionExec, rosAgArch);
				break;
			case "point_at" :
				action = new PointAt(actionExec, rosAgArch);
				break;
			case "enable_animated_speech" :
				action = new EnableAnimatedSpeech(actionExec, rosAgArch);
				break;
			case "face_human" :
				action = new FaceHuman(actionExec, rosAgArch);
				break;
			case "rotate" :
				action = new Rotate(actionExec, rosAgArch);
				break;
			case "look_at" :
				action = new LookAt(actionExec, rosAgArch);
				break;
			case "text2speech":
				action = new TextToSpeech(actionExec, rosAgArch);
				break;
			case "listen":
				action = new Listen(actionExec, rosAgArch);
				break;
			case "get_route_verbalization":
				action = new GetRouteVerba(actionExec, rosAgArch);
				break;
			case "look_at_events":
				action = new PubLookAtEvents(actionExec, rosAgArch, lookAtEventsPub);
				break;
			case "move_to":
				action = new MoveTo(actionExec, rosAgArch);
				break;
			case "engage":
				action = new Engage(actionExec, rosAgArch);
				break;
			case "terminate_interaction":
				action = new TerminateInteraction(actionExec, rosAgArch);
				break;
			case "localise":
				action = new Localise(actionExec, rosAgArch);
				break;
			case "reinit_loca":
				action = new ReinitLoca(actionExec, rosAgArch);
				break;
			case "set_param":
				action = new SetParam(actionExec, rosAgArch);
				break;
			case "configureNode":
				action = new ConfigureNode(actionExec, rosAgArch);
				break;
			case "startParameterLoaderNode":
				action = new StartParameterLoaderNode(actionExec, rosAgArch);
				break;
			case "startROSNodeGuiding":
				action = new StartROSNode(actionExec, rosAgArch);
				break;
			case "initServices":
				action = new InitServices(actionExec, rosAgArch);
				break;
			case "retryInitServices":
				action = new RetryInitServices(actionExec, rosAgArch);
				break;
			case "initGuidingAs":
				action = new InitGuidingAs(actionExec, rosAgArch);
				break;
			case "set_guiding_result":
				action = new SetGuidingResult(actionExec, rosAgArch);
				break;
			default:
				break;
		}
			
		return action;
	}
	
}
