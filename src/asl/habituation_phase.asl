// Agent habituation_phase in project supervisor
robot_place("pepper_infodesk").
task(0, guiding, "human_bidon", _).

{include("guiding.asl")}


!start(0).

/* Plans */

+!start(0): true <- 
	.verbose(2);
	rjs.jia.log_beliefs;	
	jia.robot.get_param("/guiding/robot_base/position", "List", P);
	jia.robot.get_param("/guiding/robot_base/orientation", "List", O);
	set_param("/guiding/dialogue/hwu", false);
	Rframe = map;
	move_to(Rframe,P, O);
	text2speech(introduce);
	!demo("marco_polo");
	.abolish(_[ID,add_time(_),source(_)]);
	.wait(1000);
	text2speech(other_place);
	!demo("zizzi");
	.wait(1000);
	text2speech(thanks).
	
+!demo(Place) : true <- 
	jia.robot.get_param("/guiding/perspective/robot_place", String, Rp);
	jia.robot.get_param("/guiding/robot_base/position", "List", P);
	jia.robot.get_param("/guiding/robot_base/orientation", "List", O);
	Rframe = map;
	Human = "human_bidon";
	.wait(1000);
	compute_route(Rp, Place, lambda, false);
	?route(R2);
	// TODO update quand direction n'est plus stairs
	?target_place(Target);
	get_onto_individual_info(getName, Target, verba_name);
	// if the route has an interface (a direction)
	if(.length(R2) > 3){
		.nth(2,R2,Dir);
		+direction(Dir)[ID];
		get_onto_individual_info(getName, Dir, verba_name);
	};
	if(rjs.jia.believes(direction(_))){
		?direction(Dir);
		if(jia.robot.has_mesh(Dir)){
			get_placements(Target,Dir,Human,0);
		}
	}else{
		get_placements(Target,"",Human,0);
	};
	?robot_pose(RframeC,RpositC, RorientC);
	move_to(RframeC,RpositC, RorientC);
	.abolish(point_at(_));
	+canSee(Place)[source(Human)];
	!show_target(ID);
	.abolish(point_at(_));
	if(rjs.jia.believes(direction(_))){
		!speak(ID, explain_route);
		?direction(Dir);
		+canSee(Dir)[source(Human)];
	}
	!show_direction(ID).
//	
//	move_to(Rframe,P, O);.

+!speak(ID, ToSay) : true <-
	?task(ID, _, Human, _);
	if(not rjs.jia.believes(said(ToSay,_))){
		+said(ToSay,0)[ID];
	}else{
		?said(ToSay,N);
		+said(ToSay,N+1)[ID];
	}
	text2speech(ToSay).
	
+!ask_seen(ID, Ld) : true <- true.
	
+!log_failure(ID, Subgoal, Failure, Code) : true <- 
	if(not rjs.jia.believes(failure(Subgoal, Failure, Code,_))){
		+failure(Subgoal, Failure, Code,0)[ID];
	}else{
		?failure(Subgoal, Failure, Code,N);
		+failure(Subgoal, Failure, Code,N+1)[ID];
	}.
	
