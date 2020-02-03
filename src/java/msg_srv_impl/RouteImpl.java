package msg_srv_impl;

import java.util.List;

import org.ros.internal.message.RawMessage;

import semantic_route_description_msgs.Route;

public class RouteImpl implements Route {
	
	private List<String> route;
	private String goal;
	private int cost;

	public RouteImpl() {
	}
	

	public RouteImpl(List<String> route, String goal, int cost) {
		super();
		this.route = route;
		this.goal = goal;
		this.cost = cost;
	}

	public String getGoal() {
		return goal;
	}


	public void setGoal(String goal) {
		this.goal = goal;
	}


	public int getCost() {
		return cost;
	}


	public void setCost(int cost) {
		this.cost = cost;
	}


	public RawMessage toRawMessage() {
		return null;
	}

	public List<String> getRoute() {
		return route;
	}

	public void setRoute(List<String> arg0) {
		route = arg0;
	}

}
