% set horizon of each agent based on objective
#defined time/1.
#defined horizon/2.
#defined goal_reached/2.

#const collisions = 5.

#program sum_of_cost.
horizon(A,H+D) :- agent(A), sp_length(A,H), delta(D).

#program makespan.
horizon(A,H) :- agent(A), makespan(H).

#program mapf.

#defined move/4.
#defined at/3.

time2(A,1..T) :- agent(A), horizon(A,T).
time2(1..M) :- makespan(M).
time(A,T) :- time2(A,T), T <= collisions.
time(T) :- time2(T), T <= collisions.
#show time/1.

% -------------------------------------------------------------------------

at(R,P,0) :- start(R,P), agent(R).

{ move(A,U,V,T) : edge(U,V), reach(A,V,T)} 1 :- reach(A,U,T-1), time(T).

% - move ------------------------------------------------------------------
at(R,V,T) :- move(R,_,V,T).
          :- move(R,U,_,T), not at(R,U,T-1).

% - inertia ---------------------------------------------------------------
at(R,V,T) :- at(R,V,T-1), not move(R,V,_,T), agent(R), time(T).

% - vertex collision ------------------------------------------------------
% :- { at(R,   V   ,T) : agent(R) }  > 1, vertex(   V   ), time(T).
 :- { at(R,(V,_),T) : agent(R) }  > 1, vertex((V,_)), time(T).
 
% - edge collision --------------------------------------------------------
% :- move(_,U,V,T), move(_,V,U,T), U < V.
 :- move(_,(U,_),(V,_),T), move(_,(V,_),(U,_),T), U < V, edge((U,_),(V,_)).

% - auxiliaries -------------------------------------------- redundant ----
 :- { at(R,V,T) } != 1, agent(R), time(T).
% can not be somewhere that is not reachable
:- at(A,V,T), not reach(A,V,T), time(A,T).
% dont turn 3 times in a row
%:- move(A,(U,_),(U,_),T-2), move(A,(U,_),(U,_),T-1), move(A,(U,_),(U,_),T), time(A,T), T >= 3.
% - query -----------------------------------------------------------------
%:- goal(R,P), not at(R,  P  ,horizon).
%:- goal(R,P), not at(R,(P,_),horizon).

goal_reached(R,T) :- goal(R,(V,_)), at(R,(V,_),T), time(T).


#program optimize_goals.
#maximize{1@2,R : goal_reached(R,T)}.

#program optimize_moves.
#maximize{1@1,R,T : move(R,_,_,T) }.


#show goal_reached/2.

#show.
#show at/3.
%#show move/4.
#show horizon/2.