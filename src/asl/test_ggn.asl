{ include("guiding_goal_negociation.asl")}
//"casual dining","asian","native","hamburger","fast food","pizzeria"

//listen_result(, )

!start.


+!start : true <- 
	.verbose(1); 
	rjs.jia.log_beliefs;
	!tests.

+!test(Place, Result): true <-
	-guiding_goal_nego(_,_);
	+listen_result(generic,"yes");
	!guiding_goal_negociation(0, 0,Place);
	-listen_result(_,_);
	?guiding_goal_nego(0, PlaceOnto);
	.eval(X, PlaceOnto=Result);
	if(X=false){
		.print(Place," is ", PlaceOnto, " instead of ", Result);
	}else{
		.print(Place, " OK");
	}.

-!test(Place, Result): true <- .print("no result for ",Place).

+!tests : true <-
	!test("ti ti nalle", "Ti_Ti_Nallen_Koti");
	!test("björn borg", "Bjorn_Borg");
	!test("river", "River_and_Co");
	!test("dinsko", "Din_Sko");
	!test("jack jones", "Vero_Moda_JandJ");
	!test("jewelsport", "Juvesport");
	!test("toilets", ["gf_toilet_west","corridor_toilets_west","corridor_toilets_east","ff_toilet_east","gf_toilet_east","gf_toilet_Telia","ff_toilet_west"]);
	!test("atm", ["gf_atm_west","gf_atm_east"]);
	!test("cash", ["gf_atm_west","gf_atm_east"]);
	!test("cash machine", ["gf_atm_west","gf_atm_east"]);
	!test("florist", "Makelan_Kauppapuutarna");
	+listen_result(list_places, "bella roma");
	!test("pizzeria", "Bella_Roma_and_Daddys_Diner");
	+listen_result(list_places, "linkosuo");
	!test("cafe", "Cafe_Linkosuo");
	!test("bella's roma", "Bella_Roma_and_Daddys_Diner");
	!test("liisa bakery", "Cafe_de_Lisa");
	+listen_result(list_places, "Gant");
	!test("dress", "Gant");
	+listen_result(list_places, "brothers");
	!test("pant", "Brother_Clothing");
	+listen_result(list_places, "top sport");
	!test("sport", "Top_Sport");
	!test("info point", "Information_desk");
	!test("toy store", "Kid_Zone"); // doesnt work
	!test("nissan", "Nissen");
	!test("shoemaker", "Avain__ja_suutarityo_Helsky");
	!test("beauty pedicure manna", "Kauneus__ja_jalkahoitola_Manna").
