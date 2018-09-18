display_basic_marker_test(ID) :-
  range(I, 0, 5),
  PX is 0.0,
  PY is I * 100.0 - 200.0,
  PZ is 0.0,
  R is I * 20.0,
  G is I * 40.0,
  B is I * 50.0,
  A is 1.0,
  atom_string(I, StringI),
  string_concat("BasicMarker", StringI, ID),
  display_basic_marker(I, [PX, PY, PZ, 0.0,0.0,0.0,1.0], [R,G,B,A], 1.0, ID).

range(Low, Low, High).
range(Out, Low, High) :- NewLow is Low+1, NewLow =< High, range(Out, NewLow, High).

display_trajectory_marker_test(Points) :-
  findall([0.0, I, 100.0], range(I, 0.0, 2000.0), Points),
  display_trajectory_marker(Points, [255.0, 0.0, 0.0, 1.0], [0.0, 255.0, 0.0, 1.0], "TrajectoryMarkerID").


% ,flase will force iteration, since nothing will ever be TRUE.
remove_all_marker_test(_) :-
  range(I, 0, 5),
  atom_string(I, StringI),
  string_concat("BasicMarker", StringI, ID),
  remove_marker(ID),
  remove_marker("TrajectoryMarkerID"),
  false.
