package arch.actions.robot;

import java.util.ArrayList;

import org.ros.exception.RemoteException;
import org.ros.message.Duration;
import org.ros.node.service.ServiceResponseListener;

import com.github.rosjava_actionlib.ActionClient;
import com.github.rosjava_actionlib.ActionClientListener;

import actionlib_msgs.GoalStatus;
import actionlib_msgs.GoalStatusArray;
import arch.agarch.RobotAgArch;
import dialogue_as.dialogue_actionActionFeedback;
import dialogue_as.dialogue_actionActionGoal;
import dialogue_as.dialogue_actionActionResult;
import dialogue_as.dialogue_actionGoal;
import jason.asSemantics.ActionExec;
import jason.asSyntax.ListTermImpl;
import jason.asSyntax.Term;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import std_srvs.EmptyResponse;

public class Listen extends AbstractAction implements ActionClientListener<dialogue_actionActionFeedback, dialogue_actionActionResult>{
	
	private ActionClient<dialogue_actionActionGoal, dialogue_actionActionFeedback, dialogue_actionActionResult> actionClientDialogue;
	
	dialogue_actionActionFeedback listening_fb;
	dialogue_actionActionFeedback listening_fb_prev = null;
	int count = 0;
			
	public Listen(ActionExec actionExec, AbstractROSAgArch rosAgArch,  ActionClient<dialogue_actionActionGoal, dialogue_actionActionFeedback, dialogue_actionActionResult> actionClientDialogue) {
		super(actionExec, rosAgArch);
		this.actionClientDialogue = actionClientDialogue;
	}
	
	

	@Override
	public void execute() {
		
		boolean hwu_dial = rosnode.getParameters().getBoolean("guiding/dialogue/hwu");
		if(!hwu_dial) {
			actionClientDialogue.attachListener(this);
			 boolean serverStarted;

			 logger.info("Waiting for dialogue action server to start");
	        serverStarted = actionClientDialogue.waitForActionServerToStart(new Duration(10));
	        if (serverStarted) {
	            logger.info("Action server started.\n");
	        } else {
	        	logger.info("No actionlib server found");
	        }
	        
			((RobotAgArch) rosAgArch).setInQuestion(true);
			
			ArrayList<String> words = new ArrayList<String>();
			if(actionExec.getActionTerm().getTerms().get(1).isList()) {
				for (Term term : (ListTermImpl) actionExec.getActionTerm().getTerms().get(1)) {
					words.add(term.toString().replaceAll("^\"|\"$", ""));
				}
			}else {
				words.add(actionExec.getActionTerm().getTerms().get(1).toString().replaceAll("^\"|\"$", ""));
			}
			
			dialogue_actionActionGoal listen_goal_msg = rosAgArch.getMessageFactory().newFromType(dialogue_actionActionGoal._TYPE);
			dialogue_actionGoal listen_goal = listen_goal_msg.getGoal();
			listen_goal.setSubjects(words);
			listen_goal.setEnableOnlySubject(true);
			listen_goal_msg.setGoal(listen_goal);
			actionClientDialogue.sendGoal(listen_goal_msg);
			ServiceResponseListener<std_srvs.EmptyResponse> respListener = new ServiceResponseListener<std_srvs.EmptyResponse>() {

				@Override
				public void onFailure(RemoteException e) {}

				@Override
				public void onSuccess(EmptyResponse arg0) {}
			};
			rosnode.callAsyncService("display_listening", respListener, null);
	        rosAgArch.addBelief("listening");
			
		} else {
			actionExec.setResult(true);
		}

	}

	@Override
	public void feedbackReceived(dialogue_actionActionFeedback fb) {
		listening_fb = fb;
		if (listening_fb != null & listening_fb != listening_fb_prev) {
			rosAgArch.removeBelief("not_exp_ans(_)");
			rosAgArch.addBelief("not_exp_ans(" + Integer.toString(count) + ")");
			count += 1;
			listening_fb_prev = listening_fb;
			((RobotAgArch) rosAgArch).setStartTimeAction();
		}
	}

	@Override
	public void resultReceived(dialogue_actionActionResult arg0) {
		if(arg0.getStatus().getStatus() == GoalStatus.SUCCEEDED) {
			((RobotAgArch) rosAgArch).setInQuestion(false);
			String question = actionExec.getActionTerm().getTerm(0).toString();
			((RobotAgArch) rosAgArch).endAction();
			rosAgArch.addBelief("listen_result(" + question + ",\"" + arg0.getResult().getSubject() + "\")");
			actionExec.setResult(true);
			rosAgArch.removeBelief("listening");
			ServiceResponseListener<std_srvs.EmptyResponse> respListener = new ServiceResponseListener<std_srvs.EmptyResponse>() {

				@Override
				public void onFailure(RemoteException e) {}

				@Override
				public void onSuccess(EmptyResponse arg0) {}
			};
			rosnode.callSyncService("reset_tablet", null);
			rosAgArch.actionExecuted(actionExec);
			actionClientDialogue.detachListener(this);
		}
	}

	@Override
	public void statusReceived(GoalStatusArray arg0) {
		
	}

}
