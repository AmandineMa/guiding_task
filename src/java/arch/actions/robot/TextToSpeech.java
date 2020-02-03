package arch.actions.robot;

import java.util.HashMap;
import java.util.Map;

import actionlib_msgs.GoalStatus;
import arch.agarch.AgArchGuiding;
import arch.agarch.RobotAgArch;
import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import jason.asSyntax.Literal;
import nao_interaction_msgs.SayResponse;
import rjs.arch.actions.AbstractAction;
import rjs.arch.actions.Action;
import rjs.arch.agarch.AbstractROSAgArch;
import rjs.utils.Tools;
import ros.RosNodeGuiding;
import rpn_recipe_planner_msgs.SupervisionServerInformActionResult;
import rpn_recipe_planner_msgs.SupervisionServerQueryActionResult;

public class TextToSpeech extends AbstractAction {
	
	String bel_functor;
	String bel_arg;
	
	public TextToSpeech(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}

	@Override
	public void execute() {
		Literal bel = (Literal) actionExec.getActionTerm().getTerm(0);
		bel_functor = bel.getFunctor();
		bel_arg = null;
		if (bel.getTerms().size() == 1)
			bel_arg = bel.getTerms().get(0).toString();
		boolean dialogueHWU = rosnode.getParameters().getBoolean("guiding/dialogue/hwu");
		if(bel_arg != null) {
			if(dialogueHWU)
				bel_arg = removeQuotes(bel_arg);
			else 
				bel_arg = removeQuotes(bel_arg).replaceAll("\\[", "").replaceAll("\\]", "");
		}
		String text = getText();
		if(dialogueHWU) {
			textToSpeechHWU(text.contains("?") ? true : false);
		} else {
			if(!text.equals("succeeded") )
				textToSpeechHomeMade(text);
			else
				actionExec.setResult(true);
		}
	}
	
	private void textToSpeechHWU(boolean isQuestion) {
		boolean result = true;
		if(!bel_functor.equals("where_are_u") && !bel_functor.equals("cannot_find") && !bel_functor.equals("going_to_move")
				&& !bel_functor.contains("step") && !bel_functor.contains("closer") && !bel_functor.contains("come")) {
			((RobotAgArch) rosAgArch).startAction(isQuestion);
		}
		if (isQuestion) {
			((RobotAgArch) rosAgArch).setInQuestion(true);
			((RobotAgArch) rosAgArch).setHumanAnswer(0);
			if(!bel_functor.equals("list_places")) {
				((RosNodeGuiding) rosnode).callDialogueQueryAS("clarification." + bel_functor, bel_arg);
			} else {
				((RosNodeGuiding) rosnode).callDialogueQueryAS("disambiguation", bel_arg);
			}
			SupervisionServerQueryActionResult listening_result = waitDialogueQueryAnswer(bel_functor);
			
			((RobotAgArch) rosAgArch).setInQuestion(false);
			((RobotAgArch) rosAgArch).setHumanAnswer(-1);
			result = false;
			if(listening_result != null) {
				switch(listening_result.getStatus().getStatus()) {
				case GoalStatus.SUCCEEDED:
					((RobotAgArch) rosAgArch).setHumanAnswer(1);
					result = true;
					break;
				case GoalStatus.PREEMPTED:
					actionExec.setFailureReason(new Atom("dialogue_preempted"), "dialogue preempted");
					break;
				default:
					actionExec.setFailureReason(new Atom("dialogue_failure"), "dialogue failure");
					break;
				}
			} else {
				actionExec.setFailureReason(new Atom("dialogue_no_return"), "dialogue did not returned");
			}
			
			
			rosAgArch.removeBelief("listening");

		} else {
			String sentence_code = bel_functor;
			if(!bel_functor.equals("failed") && !bel_functor.equals("succeeded"))
				sentence_code = "verbalisation." + bel_functor;
			
			((RosNodeGuiding) rosnode).callDialogueInformAS(sentence_code, bel_arg);
			
			SupervisionServerInformActionResult listening_result = waitDialogueInformAnswer();
			
			result = false;
			switch(listening_result.getStatus().getStatus()) {
			case GoalStatus.PREEMPTED:
				actionExec.setFailureReason(new Atom("dialogue_preempted"), "dialogue preempted");
				break;
			case GoalStatus.SUCCEEDED:
				result = true;
				break;
			default:
				actionExec.setFailureReason(new Atom("dialogue_failure"), "dialogue failure");
				break;
			}
		}
		Action actionPauseASR = new PauseASR(actionExec, rosAgArch);
		actionPauseASR.execute();
		actionExec.setResult(result);
		if(!bel_functor.equals("where_are_u") && !bel_functor.equals("cannot_find") && !bel_functor.equals("going_to_move")
				&& !bel_functor.contains("step") && !bel_functor.contains("closer") && !bel_functor.contains("come")) {
			((RobotAgArch) rosAgArch).endAction();
		}
	}
	
	public SupervisionServerInformActionResult waitDialogueInformAnswer() {
		SupervisionServerInformActionResult listening_result;
		do {
			listening_result = ((RosNodeGuiding) rosnode).getListeningResultInform();
			Tools.sleep(200);
		} while (listening_result == null || listening_result.getStatus().getStatus() != GoalStatus.SUCCEEDED);
		return listening_result;
		
	}
	
	public SupervisionServerQueryActionResult waitDialogueQueryAnswer(String s) {
		rosAgArch.addBelief("listening");
		SupervisionServerQueryActionResult listening_result;
		double start = rosAgArch.getRosTimeMilliSeconds();
		do {
			listening_result = ((RosNodeGuiding) rosnode).getListeningResultQuery();
			Tools.sleep(200);
		} while (listening_result == null && rosAgArch.getRosTimeSeconds() - start < 15);
		if(listening_result == null) {
			rosAgArch.addBelief("preempted("+((AgArchGuiding) rosAgArch).getTaskID()+")");
		} else {
			String resp;
			if (listening_result.getResult().getResult().equals("true")) {
				resp = "yes";
			} else if (listening_result.getResult().getResult().equals("false")) {
				resp = "no";
			} else {
				resp = listening_result.getResult().getResult();
			}
			if(listening_result.getStatus().getStatus() == GoalStatus.SUCCEEDED) {
				rosAgArch.addBelief("listen_result(" + s + ",\""+ resp + "\")");
			}
		}
		return listening_result;

	}
	
	public void textToSpeechHomeMade(String text) {
		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("text", text);
		SayResponse speak_to_resp = rosnode.callSyncService("speak_to", parameters);
		actionExec.setResult(speak_to_resp != null);
		((RobotAgArch) rosAgArch).endAction();
	}
	
	
	private String getText() {
		String text = "";
		switch (bel_functor) {
		case "hello":
			text = new String("Hello ! Nice to meet you");
			break;
		case "goodbye":
			text = new String("Goodbye");
			break;
		case "localising":
			text = new String("I am going to my charging station now.");
			break;
		case "closer":
			text = new String("Can you come closer, please");
			break;
		case "move_closer":
			text = new String("Can you take another step forward, please");
			break;
		case "thinking":
			text = new String("Wait, I'm thinking");
			break;
		case "list_places":
			text = new String("There are " + bel_arg + ". Which one do you want to go to ?");
			break;
		case "where_are_u":
			text = new String("Where are you, I cannot see you");
			break;
		case "found_again":
			text = new String("Ok I can see you again");
			break;
		case "cannot_find":
			text = new String("I cannot find you, sorry");
			break;
		case "ask_stairs":
			text = new String("It is upstairs. Are you able to climb stairs ?");
			break;
		case "ask_escalator":
			text = new String("Can you take the escalator ?");
			break;
		case "stairs":
			text = new String("I'm sorry, no route exists without stairs to go there");
			break;
		case "no_way":
			text = new String("I'm sorry, I cannot find a way to go there");
			break;
		case "able_to_see":
			text = new String("I think that you're seeing the place right now, good");
			break;
		case "explain_route":
			text = new String("Now, I'm going to explain to you the route");
			break;
		case "route_verbalization":
			text = bel_arg;
			break;
		case "route_verbalization_n_vis":
			text = "in this direction, " + bel_arg;
			break;
		case "no_place":
			text = new String("The place you asked for does not exist");
			break;
		case "get_attention":
			text = new String("Hey are you listening");
			break;
		case "continue_anyway":
			text = new String("Ok, I'll continue anyway");
			break;
		case "going_to_move":
			text = new String("I'm going to move so I can show you");
			break;
		case "step":
			text = new String("Can you make a few steps on your " + bel_arg + ", please");
			if(bel_arg.equals("left")) {
				bel_functor = "step_left";
			}else {
				bel_functor = "step_right";
			}
			bel_arg = null;
			break;
		case "step_more":
			text = new String("Can you move a bit more on your " + bel_arg + ", please");
			if(bel_arg.equals("left")) {
				bel_functor = "step_more_left";
			}else {
				bel_functor = "step_more_right";
			}
			bel_arg = null;
			break;
		case "cannot_move":
			text = new String("I'm sorry I cannot move, I'll try my best to show you from there");
			break;
		case "come":
			text = new String("Please, come in front of me");
			break;
		case "move_again":
			text = new String("I am sorry, we are going to move again");
			break;
		case "ask_explain_again":
			text = new String("Should I explain you again ?");
			break;
		case "cannot_show":
			text = new String("I am sorry, I cannot show you. I hope you will find your way");
			break;
		case "cannot_tell_seen":
			text = new String("Have you seen " + bel_arg + " ?");
			break;
		case "ask_show_again":
			text = new String("Should I show you again ?");
			break;
		case "sl_sorry":
			text = new String("I am sorry if you did not understand. I won't explain one more time.");
			break;
		case "pl_sorry":
			text = new String("I am sorry if you did not see. I won't show you again.");
			break;
		case "max_sorry":
			text = new String(
					"I am sorry, I give up, you asked me too many times something that I don't know.");
			break;
		case "tell_seen":
			text = new String("I can tell that you've seen " + bel_arg);
			break;
		case "visible_target":
			text = new String("Look, " + bel_arg + " is there");
			break;
		case "not_visible_target":
			text = new String(bel_arg + " is not visible from here but it is in this direction");
			break;
		case "hope_find_way":
			text = new String("I hope you will find your way");
			break;
		case "ask_understand":
			text = new String("Did you understand ?");
			break;
		case "generic":
			text = new String("Do you mean " + bel_arg + "?");
			break;
		case "happy_end":
			text = new String("I am happy that I was able to help you.");
			break;
		case "introduce":
			text = new String("Hello, my name is Pepper. Today, I'm taking my guide exam at Ideapark. Can you come in front of me ? I'm going to show you what I can do.");
			break;
		case "other_place":
			text = new String("I'm going to show you another place");
			break;
		case "thanks":
			text = new String("Thank you for watching me, now you can ask me for other places !");
			break;
		case "retire":
			if (bel_arg == null)
				throw new IllegalArgumentException("retire speech should have an argument");
			switch (bel_arg) {
			case "unknown_words":
				text = new String("You didn't told me a place where I can guide you. ");
				break;
			}
			if (!text.isEmpty()) {
				text = text + new String("Let's play now!");
				break;
			} else {
				text = new String("Let's play now!");
				break;
			}
		case "failed":
			if(bel_arg != null)
				text = new String(bel_arg);
			else
				text = "failed";
			break;
		case "succeeded":
			text = new String("succeeded");
			break;
		default:
			break;
		}
		return text;
	}

}
