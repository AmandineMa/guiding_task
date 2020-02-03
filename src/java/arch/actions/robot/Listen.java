package arch.actions.robot;

import java.util.ArrayList;

import actionlib_msgs.GoalStatus;
import arch.agarch.RobotAgArch;
import dialogue_as.dialogue_actionActionFeedback;
import dialogue_as.dialogue_actionActionResult;
import jason.asSemantics.ActionExec;
import jason.asSyntax.ListTermImpl;
import jason.asSyntax.Term;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import rjs.utils.Tools;
import ros.RosNodeGuiding;

public class Listen extends AbstractAction {
	
	public Listen(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}

	@Override
	public void execute() {
		boolean hwu_dial = rosnode.getParameters().getBoolean("guiding/dialogue/hwu");
		if(!hwu_dial) {
		((RobotAgArch) rosAgArch).setInQuestion(true);
		String question = actionExec.getActionTerm().getTerm(0).toString();
		ArrayList<String> words = new ArrayList<String>();
		if(actionExec.getActionTerm().getTerms().get(1).isList()) {
			for (Term term : (ListTermImpl) actionExec.getActionTerm().getTerms().get(1)) {
				words.add(term.toString().replaceAll("^\"|\"$", ""));
			}
		}else {
			words.add(actionExec.getActionTerm().getTerms().get(1).toString().replaceAll("^\"|\"$", ""));
		}
		((RosNodeGuiding) rosnode).callDialogueAS(words);
		rosAgArch.addBelief("listening");
		
		dialogue_actionActionResult listening_result;
		dialogue_actionActionFeedback listening_fb;
		dialogue_actionActionFeedback listening_fb_prev = null;
		int count = 0;
		do {
			listening_result = ((RosNodeGuiding) rosnode).getListeningResult();
			listening_fb = ((RosNodeGuiding) rosnode).getListeningFb();
			if (listening_fb != null & listening_fb != listening_fb_prev) {
				rosAgArch.removeBelief("not_exp_ans(_)");
				rosAgArch.addBelief("not_exp_ans(" + Integer.toString(count) + ")");
				count += 1;
				listening_fb_prev = listening_fb;
			}
			Tools.sleep(200);
		} while (listening_result == null || listening_result.getStatus().getStatus() != GoalStatus.SUCCEEDED);
		rosAgArch.addBelief("listen_result(" + question + ",\"" + listening_result.getResult().getSubject() + "\")");
		actionExec.setResult(true);
		rosAgArch.removeBelief("listening");
		} else {
			actionExec.setResult(true);
		}

	}

}
