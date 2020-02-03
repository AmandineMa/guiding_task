// Agent robot in project supervisor

/* Initial beliefs and rules */
landmark_to_see(Ld) :- (target_to_point(T) & T == Ld) | (dir_to_point(D) & D == Ld).

/* Initial goals */

//TODO quand suspension de tache, gestion des beliefs entre les differentes taches
//TODO check logs
//TODO change ID
//TODO make the human approach

/******* get route **********/	

+!get_optimal_route(ID): true <-
	?task(ID, guiding, Human, Place);
	// compute a route with Place which can be a place or a list of places
	// if it s a list of places, compute_route select the place with the smallest cost
	?robot_place(From);
	compute_route(From, Place, lambda, false);
	?route(R);
	// if the route has stairs, check if the human can use the stairs
	if(.substring("stairs", R)){
		// to know if the human can climb stairs
		!speak(ID, ask_stairs);
		listen(ask_stairs,["yes","no"]);
		?listen_result(ask_stairs,Word);
		if(not rjs.jia.believes(got_answer(ask_stairs,Word,_))){
			+got_answer(ask_stairs,Word,0)[ID];
		}else{
			?got_answer(ask_stairs,Word,N);
			+got_answer(ask_stairs,Word,N+1)[ID];
		}
		if(.substring(Word,yes)){
			+persona_asked(lambda)[ID];
		}else{
			+persona_asked(disabled)[ID];
			-route(_);
			// compute a new route with the persona information
			compute_route(From, Place, disabled, false);
		}		
	}
	?route(R2);
	// TODO update quand direction n'est plus stairs
	?target_place(Target);
	get_onto_individual_info(getName, Target, verba_name);
	// if the route has an interface (a direction)
	if(.length(R2) > 3){
		.nth(2,R2,Dir);
		+direction(Dir)[ID];
		get_onto_individual_info(getName, Dir, verba_name);
	}.

	
-!get_optimal_route(ID)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(Msg)]: true <-	
	if(.substring(wo_stairs, Failure)){
		!speak(ID, stairs);
	}else{
		!speak(ID, no_way);
	}
	!log_failure(ID, get_optimal_route, Failure, Code);
	!drop_current_task(ID, get_optimal_route, Msg, Code);.
	
/*******  go to see target **********/	

//^!go_to_see_target(ID)[state(S)] : S == started | S == resumed <- -monitoring(_, _)[add_time(_), source(self)]. 
//
//^!go_to_see_target(ID)[state(S)] : S == finished | S == failed <- ?task(ID, _, Human, _); +monitoring(ID, Human).

+!go_to_see_target(ID) : true <-
	?task(ID, guiding, Human, _);
	!get_placements(ID);
	?ld_to_point;
	!be_at_good_pos(ID).
	
-!go_to_see_target(ID)[Failure, code(Code),code_line(_),code_src(_),error(_),error_msg(_)] : true <- 
	!log_failure(ID, go_to_see_target, Failure, Code).
	
+!get_placements(ID): not no_mesh <-
	?task(ID, guiding, Human, _);
	?target_place(TargetLD);
	.concat("human_", Human, HTF);
	if(jia.robot.has_mesh(TargetLD)){
		// if there is a direction
		if(rjs.jia.believes(direction(_))){
			?direction(Dir);
			if(jia.robot.has_mesh(Dir)){
				get_placements(TargetLD,Dir,HTF,0);
			}
		}else{
			get_placements(TargetLD,"",HTF,0);
		}
	}else{
		if(rjs.jia.believes(direction(_))){
			?direction(Dir);
			if(jia.robot.has_mesh(Dir)){
				get_placements(Dir,"",HTF,1);
			}
		}
	}.
	
-!get_placements(ID)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_), source(self)] : true <- 
	!log_failure(ID, get_placements, Failure, Code).
	
+!be_at_good_pos(ID) : true <- 
	?task(ID, guiding, Human, _);
	?robot_pose(RframeC,RpositC, RorientC);
	jia.robot.publish_marker(RframeC,RpositC, yellow);
	?human_pose(Hframe,Hposit,_);
	jia.robot.publish_marker(Hframe, Hposit, blue);
	if(rjs.jia.believes(robot_move(_,_, _))){
		?robot_move(Rframe,Rposit, Rorient);
	}else{
		?robot_turn(Rframe,Rposit, Rorient);
	}
	if(rjs.jia.believes(human_first(_))){
		.concat("human_", Human, HTF);
		?human_first(Side);
		+step;
		if(rjs.tf.get_transform(map, HTF, Point,_)){
			+dist_to_goal(HTF,Point,Rposit);
		}
		!speak(ID, step(Side));
		jia.robot.get_param("/guiding/tuning_param/human_move_first_dist_th", "Double", Dist);
		!check_dist(ID, HTF, Rposit, Dist, false);
		-step;
		rjs.jia.reset_att_counter(check_dist);
	}
	.wait(800);
	if(rjs.jia.believes(robot_move(_,_, _))){
		!speak(ID, going_to_move);
	}
	-monitoring(_, _);
	human_to_monitor(""); 
	+move(started)[ID];
	if(rjs.tf.get_transform(map, "base_footprint", PointR,_)){
		+dist_to_goal(_,PointR,Rposit);
	}
	move_to(Rframe,Rposit, Rorient);
	+move(over)[ID];
	!wait_human(ID);
	++monitoring(ID, Human).
	
-step : true <- -dist_to_goal(_,_,_).
+move(over) : true <- -dist_to_goal(_,_,_).

@cd[max_attempts(10)]+!check_dist(ID, HTF, Point, Dist, Bool) : true <- 
	rjs.tf.is_dist_human2point_sup(HTF, Point, Dist, Result);
	if(.substring(Result, Bool)){
		.wait(200);
		!check_dist(ID, HTF, Point, Dist, Bool);
	}.
	
-!check_dist(ID, HTF, Point, Dist, Bool) : not adjust <- 
	?human_first(Side);
	!repeat_move(ID, HTF, Point, Dist, Bool, Side).
	
-!check_dist(ID, HTF, Point, Dist, Bool) : adjust <- 
	!repeat_move(ID, HTF, Point, Dist, Bool).	
	
@rm[max_attempts(2)]+!repeat_move(ID, HTF, Point, Dist, Bool, Side) : true <-
	!speak(ID, step(Side));
	rjs.jia.reset_att_counter(check_dist);
	!check_dist(ID, HTF, Point, Dist, Bool).
	
@rm2[max_attempts(2)]+!repeat_move(ID, HTF, Point, Dist, Bool) : true <-
	!speak(ID, closer);
	rjs.jia.reset_att_counter(check_dist);
	!check_dist(ID, HTF, Point, Dist, Bool).
	
-!repeat_move(ID, HTF, Point, Dist, Bool, Side) : true <- 
	!speak(ID, cannot_move); 
	!log_failure(ID, repeat_move, cannot_move, _);
	-step;
	.succeed_goal(be_at_good_pos(ID)).
	
-!repeat_move(ID, HTF, Point, Dist, Bool) : true <- 
	!log_failure(ID, repeat_move, adjust, _);
	if(rjs.jia.believes(dir_to_point(_))){
		?dir_to_point(D);
		if(not jia.robot.can_be_visible(HTF, D)){
			-dir_to_point(D);
		}
	}
	if(rjs.jia.believes(target_to_point(_))){
		?target_to_point(T);
		if(not jia.robot.can_be_visible(HTF, T)){
			-target_to_point(T);
		}
	}
	-adjust;
	.succeed_goal(be_at_good_pos(ID)).

-!be_at_good_pos(ID)[Failure, code(Code),code_line(_),code_src(_),error(_),error_msg(_)] : true <-
	if(not .substring(robot_pose, Code)){
		!log_failure(ID, be_at_good_pos, Failure, Code);
	}.
 
@wh[max_attempts(3)]+!wait_human(ID) : true <- 
	+wait_human;
	human_to_monitor("");
	?task(ID, guiding, Human, _);
	?robot_pose(Rframe,Rposit, Rorient);
	?human_pose(Hframe,Hposit,_);
	.concat("human_", Human, HTF);
	if(rjs.tf.get_transform(map, HTF, Point,_)){
		.nth(2, Point, Z);
		if(Z > 1.8){
			Zbis=1.8;
		}else{
			Zbis=Z;
		}	
		rjs.jia.replace(2, Hposit, Zbis, Pointf);
		jia.robot.publish_marker(Hframe, Pointf, blue);
		-look_at(look);
		look_at(Hframe,Pointf,true);
		.wait(isPerceiving(Human),4000);
		++monitoring(ID, Human);
		+after_move_status(human_found)[ID];
		.wait(look_at(look),4000);
		look_at_events(human_perceived);
		-wait_human;
		!check_pos(ID, Human);
	}else{
		!log_failure(ID, wait_human, no_transform, rjs.tf.get_transform(map, HTF, Point,_));
	}.
	
@cp[max_attempts(3)]+!check_pos(ID, Human): true <-
	.concat("human_", Human, HGTF);
	human_to_monitor(HGTF);
	if(rjs.jia.believes(dir_to_point(_))){
		?dir_to_point(D);
		.concat("human_", Human, HTF);
		can_be_visible(HTF, D);
		+visible(direction, D, true)[ID];
	}
	if(rjs.jia.believes(target_to_point(_))){
		?target_to_point(T);
		.concat("human_", Human, HTF);
		can_be_visible(HTF, T);
		+visible(target, T, true)[ID];
	}.

-!check_pos(ID, Human)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)] : true <- 
	if(.substring(not_visible,Failure)){
		.concat("human_", Human, HTF);
		?human_pose(Hframe,Hposit,_);
		jia.robot.get_param("/guiding/tuning_param/human_move_not_visible", "Double", Dist);
		rjs.tf.is_dist_human2point_sup(HTF, Hposit, Dist, Result);
		
		if(.substring(Result, true) & rjs.jia.believes(dir_to_point(_))){
			?dir_to_point(D);
			.concat("human_", Human, HTF);
			if(not jia.robot.can_be_visible(HTF, D)){
				if(not rjs.jia.believes(visible(direction, D, false, _))){
					+visible(direction, D, false, 0)[ID];
				}else{
					?visible(direction, D, false, N);
					+visible(direction, D, false, N+1)[ID];
				}
				!adjust_human_pos(ID, Human);
//				!check_pos(ID, Human);
			}
		}elif(.substring(Result, true) & rjs.jia.believes(target_to_point(_))){
			?target_to_point(T);
			.concat("human_", Human, HTF);
			if(not jia.robot.can_be_visible(HTF, T)){
				if(not rjs.jia.believes(visible(target, T, false, _))){
					+visible(target, T, false, 0)[ID];
				}else{
					?visible(target, T, false, N);
					+visible(target, T, false, N+1)[ID];
				}
				!adjust_human_pos(ID, Human);
//				!check_pos(ID, Human);
			}
		}
	}else{
		!log_failure(ID, check_pos, Failure, Code);
	}.
	
+!adjust_human_pos(ID, Human) : true <-
	?human_pose(Hframe,Hposit,_);
	.concat("human_", Human, HTF);
	+adjust;
	if(rjs.tf.get_transform(map, HTF, Point,_)){
		+dist_to_goal(HTF,Point,Hposit);
	}
	!speak(ID, closer);
	.concat("human_", Human, HTF);
	jia.robot.get_param("/guiding/tuning_param/human_move_not_visible", "Double", Dist);
	!check_dist(ID, HTF, Hposit, Dist, true);
	-adjust;
	rjs.jia.reset_att_counter(check_dist).
	
-adjust : true <- 
	-dist_to_goal(_,_,_);
	look_at_events(stop_look_at).

-!adjust_human_pos(ID, Human)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)] : true <-
	-adjust(_);
	!log_failure(ID, check_pos, Failure, Code).
	
-!wait_human(ID)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)] : true <-
	?task(ID, guiding, Human, _);
	// TODO meme erreur que pour isPerceiving, ne permet pas de differencier les deux
	if(.substring(wait_timeout, Error) & .substring(isPerceiving, Code)){
		!wait_look_at;
		look_at_events(stop_look_at);
		if(not rjs.jia.believes(after_move_status(call_human,_))){
			+after_move_status(call_human,0)[ID];
		}else{
			?after_move_status(call_human,N);
			+after_move_status(call_human,N+1)[ID];
		}
		!speak(ID, come);
		!wait_human(ID);
	}elif(.substring(wait_timeout, Error) & .substring(look_at, Code)){
		look_at_events(stop_look_at);
		+after_move_status(look_at_not_received)[ID];
		!log_failure(ID, look_at, not_received, look_at(look));
		-wait_human;
		!check_pos(ID, Human);
	}elif(.substring(max_attempts,Error)){
		-look_at(look);
		look_at_events(stop_look_at);
		+after_move_status(human_not_found)[ID];
		!speak(ID,cannot_find); 
		!drop_current_task(ID, wait_human, max_attempts, "wait too long");
	}else{
//		!drop_current_task(ID, wait_human, Failure, Code);
		!log_failure(ID, wait_human, Failure, Code);
	}.

+!wait_look_at : true <- .wait(look_at(look),4000).
-!wait_look_at : true <- true.
	
/*******  show landmarks **********/

@sl[max_attempts(3)]+!show_landmarks(ID) : true <- 
	?task(ID, guiding, Human, _);
//	++need_attentive_human(Human);
	jia.robot.get_param("/guiding/dialogue/hwu", "Boolean", Dialogue);
	if(Dialogue == true){
		enable_animated_speech(false);
	}
	.abolish(point_at(_));
	!show_target(ID);
	.abolish(point_at(_));
	if(rjs.jia.believes(direction(_))){
		!speak(ID, explain_route);
		?direction(Dir);
	}
	!show_direction(ID); 
	!ask_understood(ID).
//	if(rjs.jia.believes(direction_explained) & not rjs.jia.believes(ask_seen(Dir))){
//		!speak(ID, ask_understand);
//		listen(ask_understand,["yes","no"]);
//		?listen_result(ask_understand,Word1);
//		if(not rjs.jia.believes(got_answer(ask_understand,Word1,_))){
//			+got_answer(ask_understand,Word1,0)[ID];
//		}else{
//			?got_answer(ask_understand,Word1,N);
//			+got_answer(ask_understand,Word1,N+1)[ID];
//		}
//		if(.substring(Word1,yes)){
//			!speak(ID, happy_end);
//		}else{
//			!speak(ID, ask_explain_again);
//			listen(ask_explain_again,["yes","no"]);
//			?listen_result(ask_explain_again,Word2);
//			if(not rjs.jia.believes(got_answer(ask_explain_again,Word2,_))){
//				+got_answer(ask_explain_again,Word2,0)[ID];
//			}else{
//				?got_answer(ask_explain_again,Word2,N);
//				+got_answer(ask_explain_again,Word2,N+1)[ID];
//			}
//			if(.substring(Word2,yes)){
//				!show_landmarks(ID);
//			}else{
//				!speak(ID, hope_find_way);
//			}
//		}
	
	
+!ask_understood(ID) : direction_said & direction(Dir) & not ask_seen(Dir) <-
	.wait(800);
	if(Dialogue == true){
		enable_animated_speech(true);
	}
	!speak(ID, ask_understand);
	listen(ask_understand,["yes","no"]);
	?listen_result(ask_understand,Word1);
	if(not rjs.jia.believes(got_answer(ask_understand,Word1,_))){
		+got_answer(ask_understand,Word1,0)[ID];
	}else{
		?got_answer(ask_understand,Word1,N);
		+got_answer(ask_understand,Word1,N+1)[ID];
	}
	if(.substring(Word1,yes)){
		+direction_explained[ID];
		!speak(ID, happy_end);
	}else{
		!speak(ID, ask_explain_again);
		listen(ask_explain_again,["yes","no"]);
		?listen_result(ask_explain_again,Word2);
		if(not rjs.jia.believes(got_answer(ask_explain_again,Word2,_))){
			+got_answer(ask_explain_again,Word2,0)[ID];
		}else{
			?got_answer(ask_explain_again,Word2,N);
			+got_answer(ask_explain_again,Word2,N+1)[ID];
		}
		if(.substring(Word2,yes)){
			!show_direction(ID);
			!ask_understood(ID);
		}else{
			!speak(ID, hope_find_way);
		}
	}.
	
+!ask_understood(ID) : not ( direction_explained & direction(Dir) & not ask_seen(Dir) ) <-
	if(rjs.jia.believes(target_explained)){
		!speak(ID, happy_end);
	}else{
		!speak(ID, cannot_show);
	}.
	
	
-!show_landmarks(ID)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)] : true <-
	if(.substring(Error,max_attempts)){
		!speak(ID,sl_sorry); 
		!drop_current_task(ID, show_landmarks, max_attempts, multiple_wrong_answers);
	}else{
		!log_failure(ID, show_landmarks, Failure, Code);
//		!drop_current_task(ID, show_landmarks, Failure, Code);
	}.


+!show_target(ID) : true <-
	?target_place(TargetPlace);
	// if cannot transform, leaves the plan
	rjs.tf.can_transform(map, TargetPlace);
	!point_look_at(ID, TargetPlace);
//	+target_explained[ID];
	rjs.jia.reset_att_counter(point_look_at).
	

-!show_target(ID)[Failure, code(Code),code_line(_),code_src(_),error(_),error_msg(_)] : true <-
	if(not .substring(can_transform, Code)){
		!log_failure(ID, show_target, Failure, Code);
	}else{
		?target_place(TargetPlace);
		if(rjs.jia.believes(target_place(TargetPlace))){
 			+target_verba[ID];
	 	}elif(rjs.jia.believes(direction(TargetPlace))){
	 		+direction_verba[ID];
	 	};
		!verba(ID, TargetPlace);
//		+target_explained[ID];
	}.
	
+!show_direction(ID) : true <-
	?direction(D);
	rjs.tf.can_transform(map, D);
	!point_look_at(ID, D);
//	+direction_explained[ID];
	rjs.jia.reset_att_counter(point_look_at).

-!show_direction(ID)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)] : true <-
	?task(ID, guiding, Human, _);
	if(.substring(can_transform, Code)){
		?direction(D);
		if(rjs.jia.believes(target_place(D))){
 			+target_verba[ID];
	 	}elif(rjs.jia.believes(direction(D))){
	 		+direction_verba[ID];
	 	};
		!verba(ID, D);
//		+direction_explained[ID];
	}elif(not .substring(test_goal_failed, Error)){
		!log_failure(ID, show_direction, Failure, Code);
	}.

// TODO handle timeout point_at
@pl_l[max_attempts(3), atomic_r]+!point_look_at(ID, Ld) : landmark_to_see(Ld) <-
	?task(ID, guiding, Human, _);
	if(rjs.jia.believes(target_place(Ld))){
 		+target_verba[ID];
 	}elif(rjs.jia.believes(direction(Ld))){
 		+direction_verba[ID];
 	};
	point_at(Ld,false,true);
	.wait(point_at(point),10000);
	-point_at(point);
	!verba(ID, Ld);
	.wait(point_at(finished),6000);
	-point_at(finished);
	?(canSee(Ld)[source(Human)] | hasSeen(Ld)[source(Human)]).
//	?verba_name(Ld,Verba);
//	!speak(ID, tell_seen(Verba)).


@pl_nl[max_attempts(3), atomic_r]+!point_look_at(ID, Ld) : not landmark_to_see(Ld) <-
	?task(ID, guiding, Human, _);
	if(rjs.jia.believes(target_place(Ld))){
 		+target_verba[ID];
 	}elif(rjs.jia.believes(direction(Ld))){
 		+direction_verba[ID];
 	};
	point_at(Ld,false,true);
	.wait(point_at(point),10000);
	-point_at(point);
	jia.robot.get_param("/guiding/dialogue/hwu", "Boolean", Dialogue);
	!verba(ID, Ld);
	.wait(point_at(finished),6000);
	-point_at(finished).


-!point_look_at(ID, Ld)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)] : true <-
	?task(ID, guiding, Human, _);
	if(.substring(test_goal_failed, Error) |  .substring(point_at(finished),Code)){
		!ask_seen(ID,Ld);
	}elif(.substring(Error,max_attempts)){
		!speak(ID,pl_sorry); 
		rjs.jia.reset_att_counter(point_look_at);
	}elif(.substring(point_at(point),Code)){
		.print("time out time ",T2);
		!log_failure(ID, point_look_at, Code, not_received);
		!verba(ID, Ld);
	}else{
		!log_failure(ID, point_look_at, Failure, Code);
		!verba(ID, Ld);
	}.
	
+!ask_seen(ID, Ld) : true <-
	+ask_seen(Ld)[ID];
	?task(ID, guiding, Human, _);
	?verba_name(Ld,Verba);
	!speak(ID, cannot_tell_seen(Verba));
	listen(cannot_tell_seen,["yes","no"]);
	?listen_result(cannot_tell_seen, Word1);
	if(not rjs.jia.believes(got_answer(cannot_tell_seen,Word1,_))){
		+got_answer(cannot_tell_seen,Word1,0)[ID];
	}else{
		?got_answer(cannot_tell_seen,Word1,N);
		+got_answer(cannot_tell_seen,Word1,N+1)[ID];
	}
	if(.substring(Word1,no)){
		!speak(ID, ask_show_again(Ld));
		listen(ask_show_again,["yes","no"]);
		?listen_result(ask_show_again, Word2);
		if(not rjs.jia.believes(got_answer(ask_show_again,Word2,_))){
			+got_answer(ask_show_again,Word2,0)[ID];
		}else{
			?got_answer(ask_show_again,Word2,N);
			+got_answer(ask_show_again,Word2,N+1)[ID];
		}
		if(.substring(Word2,yes)){
			// TODO add max_attempts
			!point_look_at(ID,Ld);
		}else{
			!speak(ID, hope_find_way);
			+did_not_see[ID];
		}
	}else{
		+ld_seen(Ld)[ID];
	}.
	
-!ask_seen(ID, Ld)[Failure, code(Code),code_line(_),code_src(_),error(_),error_msg(_)]: true <-
  	!log_failure(ID, ask_seen, Failure, Code).
  	
 +!verba(ID,Place) : true <-
// 	if(rjs.jia.believes(target_place(Place))){
// 		+target_verba[ID];
// 	}elif(rjs.jia.believes(direction(Place))){
// 		+direction_verba[ID];
// 	};
 	!verbalization(ID, Place);
 	if(rjs.jia.believes(target_place(Place))){
 		+target_explained[ID];
 	}elif(rjs.jia.believes(direction(Place))){
 		+direction_said[ID];
 	}.

+!verbalization(ID, Place) : target_to_point(T) & T == Place & direction(_) <-
	?task(ID, guiding, Human, _);
	?verba_name(Place, Name);
	!speak(ID, visible_target(Name)).
	
+!verbalization(ID, Place) : not target_to_point(_) &  direction(D) & Place \== D <-
	?task(ID, guiding, Human, _);
	?verba_name(Place, Name);
	!speak(ID, not_visible_target(Name)).

+!verbalization(ID, Place) : ( direction(D) & Place == D & dir_to_point(D)) | (target_to_point(T) & T == Place & not direction(_)) <-
	?task(ID, guiding, Human, _);
	!get_verba(ID);
	?verbalization(RouteVerba);
	!speak(ID, route_verbalization(RouteVerba)).

+!verbalization(ID, Place) : ( direction(D) & Place == D & not dir_to_point(D) ) | ( not target_to_point(_) & not direction(_)) <-
	?task(ID, guiding, Human, _);
	!get_verba(ID);
	?verbalization(RouteVerba);
	!speak(ID, route_verbalization_n_vis(RouteVerba)).
	
+!get_verba(ID) : true <-
	?route(Route);
	?robot_place(RobotPlace);
	?target_place(FinaleP);
	get_route_verbalization(Route, RobotPlace, FinaleP).
	
-!verbalization(ID, Place)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)] : true <-
	!drop_current_task(ID, verbalization, Failure, Code).

		
/********* **********/		

  	