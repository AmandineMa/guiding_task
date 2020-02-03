package arch.actions.robot;

import org.ros.node.topic.Publisher;

import jason.asSemantics.ActionExec;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;

public class HumanToMonitor extends AbstractAction {
	
	private Publisher<std_msgs.String> humanToMonitorPub; 

	public HumanToMonitor(ActionExec actionExec, AbstractROSAgArch rosAgArch, Publisher<std_msgs.String> humanToMonitorPub) {
		super(actionExec, rosAgArch);
		this.humanToMonitorPub = humanToMonitorPub;
		setSync(true);
	}

	@Override
	public void execute() {
		String param = removeQuotes(actionExec.getActionTerm().getTerm(0).toString());
		std_msgs.String str = humanToMonitorPub.newMessage();
		str.setData(param);
		humanToMonitorPub.publish(str);
		actionExec.setResult(true);
	}	

}
