package arch.actions.ros;

import jason.asSemantics.ActionExec;
import rjs.arch.actions.ros.AbstractStartROSNode;
import rjs.arch.agarch.AbstractROSAgArch;
import ros.RosNodeGuiding;

public class StartROSNode extends AbstractStartROSNode {
	
//	private RosNodeGuiding rosnode;

	public StartROSNode(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}

	@Override
	public void execute() {
		rosnode = new RosNodeGuiding("supervisor");
		configRosnode();
	}

}
