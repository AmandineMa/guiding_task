package msg_srv_impl;

import java.util.ArrayList;
import java.util.List;

import org.ros.internal.message.RawMessage;

import semantic_route_description_msgs.Route;
import semantic_route_description_msgs.SemanticRouteResponse;

public class SemanticRouteResponseImpl implements SemanticRouteResponse {
	
	private float[] costs;
	private List<String> goals;
	private List<Route> routes;
	private int code;

	public SemanticRouteResponseImpl() {
	}
	
	
	public SemanticRouteResponseImpl(float[] costs, List<String> goals, List<Route> routes) {
		this.costs = costs;
		this.goals = new ArrayList<String>(goals);
		this.routes = new ArrayList<Route>(routes);
	}



	public RawMessage toRawMessage() {
		// TODO Auto-generated method stub
		return null;
	}

	public float[] getCosts() {
		return costs;
	}

	public List<String> getGoals() {
		return goals;
	}

	public List<Route> getRoutes() {
		return routes;
	}

	public void setCosts(float[] arg0) {
		costs = arg0;
	}

	public void setGoals(List<String> arg0) {
		goals = new ArrayList<String>(arg0);
	}

	public void setRoutes(List<Route> arg0) {
		routes = new ArrayList<Route>(arg0);
	}
	
	public void setCode(int arg0) {
		code = arg0;
	}
	
	public int getCode() {
		return code;
	}

}
