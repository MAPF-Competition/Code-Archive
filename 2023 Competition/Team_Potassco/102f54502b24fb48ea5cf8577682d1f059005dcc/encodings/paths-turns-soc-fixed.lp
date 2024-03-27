% set horizon of each agent based on objective
#defined time/1.
#defined goal_reached/2.

#const collisions = 5.

#defined move/4.
#defined at/3.


time(A,T) :- reach(A,_,T).
time(1..T) :- time(_,T).
last(T) :- time(T), not time(T+1).
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


%goal_reached(R,T) :- goal(R,(V,_)), at(R,(V,_),T), time(T).
%#maximize{1@2,R : goal_reached(R,T)}.

%#program optimize_moves.
%#maximize{1@1,R,T : move(R,_,_,T) }.

%#heuristic move(R,U,V,T) : dist(R,U,DU), dist(R,V,DV), time(T), reach(R,U,T-1), reach(R,V,T), edge(U,V). [1000 - |DU-DV|,factor]
%#heuristic move(R,U,V,T). [1,sign]
#minimize{D,R : at(R,V,T), last(T), dist(R,V,D)}.

%#show goal_reached/2.

#show.
#show at/3.
%#show start/2.
%#show agent/1.
%#show move/4.
