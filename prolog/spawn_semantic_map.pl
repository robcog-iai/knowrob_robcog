spawn_semantic_map(Map, Objects, Constraints, Relations) :-
  findall(RawName, spawn_models(Map ,RawName), Objects),
  findall(Id, spawn_constraints(Map, Id), Constraints),
  findall([Parent, Child], spawn_relations(Map, Parent, Child), Relations ).

spawn_models(Map, Raw) :-
   map_root_object(Map, ObjectToSpawn),
   rdf_has(ObjectToSpawn, knowrob:pose, _),
   spawn_model(ObjectToSpawn, Raw).

spawn_model(ObjectToSpawn, Raw) :-
  current_object_pose(ObjectToSpawn, Pose),
  rdf_has(ObjectToSpawn, rdf:type, Name),
  not(rdf_equal(Name, owl:'NamedIndividual')),
  remove_namespace(Name, Raw),
  remove_namespace(ObjectToSpawn, Id),
  spawn_model(Id, Raw, Pose).


% @param Pose  The pose term [atom Reference, atom Target, [float x,y,z], [float qx,qy,qz,qw]]
spawn_model(Id, Name, Pose) :-
  Pose = [_, _, Location, Quaternion],
  Location = [LX, LY, LZ],
  Quaternion = [QX, QY, QZ, QW],
  spawn_model(Id, Name, LX, LY, LZ, QX, QY, QZ, QW).


remove_namespace(WithNS, Raw) :-
  split_string(WithNS, "#", '"', New ),
  New = [_,Raw].


spawn_constraints(Map, Id) :-
  map_root_object(Map, Constraint),
  rdf_has(Constraint, rdf:type, Name),
  not(rdf_equal(Name, owl:'NamedIndividual')),
  rdf_equal(Name, knowrob:'Constraint'),
  get_constraint_properties(Constraint, Parent, Child, LinConst, AngConst, Pose),
  get_linear_constraint_properties(LinConst, LinProperties),
  get_angular_constraint_properties(AngConst, AngProperties),
  remove_namespace(Constraint, Id),
  spawn_constraint(Parent, Child, LinProperties, AngProperties, Pose).


spawn_relations(Map, Parent, Child) :-
  map_root_object(Map, ParentNS),
  rdf_has(ParentNS, rdf:type, Name),
  not(rdf_equal(Name, owl:'NamedIndividual')),
  rdf_has(ParentNS, knowrob:child, ChildNS),
  remove_namespace(ParentNS, Parent),
  remove_namespace(ChildNS, Child),
  attach_model_to_parent(Parent, Child).


spawn_constraint(Parent, Child, LinProperties, AngProperties, Pose) :-
  remove_namespace(Parent, ParentId),
  remove_namespace(Child, ChildId),
  Pose = [_, _, Location, Quaternion],
  Location = [LX, LY, LZ],
  Quaternion = [QX, QY, QZ, QW],
  LinProperties = [LinMotions, LinLimit, LinSoftConstraint, LinStiffness, LinDamping],
  LinMotions = [LMX, LMY, LMZ],
  AngProperties =  [AngMotions, AngLimits,
   AngSwingSoftConstraint, AngSwingStiffness, AngSwingDamping,
   AngTwistSoftConstraint, AngTwistStiffness, AngTwistDamping],
  AngMotions = [S1M, S2M, TM],
  AngLimits = [S1L, S2L, TL],
  spawn_constraint([ParentId, ChildId,
   LX, LY, LZ, QX, QY, QZ, QW,
   LMX, LMY, LMZ, LinLimit, LinSoftConstraint, LinStiffness, LinDamping,
   S1M, S2M, TM, S1L, S2L, TL,
   AngSwingSoftConstraint, AngSwingStiffness, AngSwingDamping,
   AngTwistSoftConstraint, AngTwistStiffness, AngTwistDamping]).


get_constraint_properties(Constraint, Parent, Child, LinConst, AngConst, Pose) :-
  rdf_has(Constraint, knowrob:parent, Parent),
  rdf_has(Constraint, knowrob:child, Child),
  rdf_has(Constraint, knowrob:linearConstraint, LinConst),
  rdf_has(Constraint, knowrob:angularConstraint, AngConst),
  current_object_pose(Constraint, Pose).


get_linear_constraint_properties(LinConst, Properties) :-
  rdf_has(LinConst, knowrob:xMotion, literal(type(xsd:integer, AXM))),
  rdf_has(LinConst, knowrob:yMotion, literal(type(xsd:integer, AYM))),
  rdf_has(LinConst, knowrob:zMotion, literal(type(xsd:integer, AZM))),
  atom_number(AXM, XM), atom_number(AYM, YM), atom_number(AZM, ZM),
  Motions = [XM, YM, ZM],
  rdf_has(LinConst, knowrob:limit, literal(type(xsd:float, ALimit))),
  rdf_has(LinConst, knowrob:softConstraint, literal(type(xsd:boolean, SoftConstraint))),
  rdf_has(LinConst, knowrob:stiffness, literal(type(xsd:float, AStiffness))),
  rdf_has(LinConst, knowrob:damping, literal(type(xsd:float, ADamping))),
  atom_number(ALimit, Limit), atom_number(AStiffness, Stiffness), atom_number(ADamping, Damping),
  Properties = [Motions, Limit, SoftConstraint, Stiffness, Damping].

get_angular_constraint_properties(AngConst, Properties) :-
  rdf_has(AngConst, knowrob:swing1Motion, literal(type(xsd:integer, AS1M))),
  rdf_has(AngConst, knowrob:swing2Motion, literal(type(xsd:integer, AS2M))),
  rdf_has(AngConst, knowrob:twistMotion, literal(type(xsd:integer, ATM))),
    atom_number(AS1M, S1M), atom_number(AS2M, S2M), atom_number(ATM, TM),
  Motions = [S1M, S2M, TM],

  rdf_has(AngConst, knowrob:swing1Limit, literal(type(xsd:float, AS1L))),
  rdf_has(AngConst, knowrob:swing2Limit, literal(type(xsd:float, AS2L))),
  rdf_has(AngConst, knowrob:twistLimit, literal(type(xsd:float, ATL))),
  atom_number(AS1L, S1L), atom_number(AS2L, S2L), atom_number(ATL, TL),
  Limits = [S1L, S2L, TL],

  rdf_has(AngConst, knowrob:softSwingConstraint, literal(type(xsd:boolean, SwingSoftConstraint))),
  rdf_has(AngConst, knowrob:swingStiffness, literal(type(xsd:float, ASwingStiffness))),
  rdf_has(AngConst, knowrob:swingDamping, literal(type(xsd:float, ASwingDamping))),
  atom_number(ASwingStiffness, SwingStiffness), atom_number(ASwingDamping, SwingDamping),

  rdf_has(AngConst, knowrob:softTwistConstraint, literal(type(xsd:boolean, TwistSoftConstraint))),
  rdf_has(AngConst, knowrob:twistStiffness, literal(type(xsd:float, ATwistStiffness))),
  rdf_has(AngConst, knowrob:twistDamping, literal(type(xsd:float, ATwistDamping))),
  atom_number(ATwistStiffness, TwistStiffness), atom_number(ATwistDamping, TwistDamping),

  Properties = [Motions, Limits,
   SwingSoftConstraint, SwingStiffness, SwingDamping,
   TwistSoftConstraint, TwistStiffness, TwistDamping].
