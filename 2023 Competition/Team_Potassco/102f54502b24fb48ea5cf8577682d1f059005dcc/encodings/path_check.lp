% the robot does not move along an edge
fail :- at(R,V,T), at(R,U,T+1), not edge(U,V).

#show fail/0.
