package arch.agarch;

public class SupervisorAgArch extends AgArchGuiding {
	private String currentGoal = null;

	public String getCurrentGoal() {
		return currentGoal;
	}

	public void setCurrentGoal(String currentGoal) {
		this.currentGoal = currentGoal;
	}
}
