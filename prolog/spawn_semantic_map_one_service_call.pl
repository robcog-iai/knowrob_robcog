spawn_semantic_map(Map) :-
  spawn_semantic_map(Map, _, _, _).

spawn_semantic_map(Map, Models, Constraints, Relations) :-
  findall(ModelProperties, spawn_models(Map, ModelProperties), Models),
  findall(ConstraintProperties, spawn_constraints(Map, ConstraintProperties), Constraints),
  findall([Parent, Child], spawn_relations(Map, Parent, Child), Relations),
  spawn_semantic_map(Models, Constraints, Relations).

spawn_models(Map, ModelProperties) :-
  map_root_object(Map, ObjectToSpawn),
  rdf_has(ObjectToSpawn, knowrob:pose, _),
  current_object_pose(ObjectToSpawn, Pose),
  rdf_has(ObjectToSpawn, rdf:type, Name),
  not(rdf_equal(Name, owl:'NamedIndividual')),
  not(rdf_equal(Name, knowrob:'Constraint')),
  rdf_has(ObjectToSpawn, knowrob:tagsData,  literal(type(xsd:string, TagsData))),
  rdf_has(ObjectToSpawn, knowrob:mobility,  literal(type(xsd:string, Mobility))),
  rdf_has(ObjectToSpawn, knowrob:mass,  literal(type(xsd:float, MassAtom))), atom_number(MassAtom, Mass),
  rdf_has(ObjectToSpawn, knowrob:overlapEvents,  literal(type(xsd:boolean, OverlapEvents))),
  rdf_has(ObjectToSpawn, knowrob:gravity,  literal(type(xsd:boolean, Gravity))),
  remove_namespace(Name, Raw),
  remove_namespace(ObjectToSpawn, Id),
  Pose = [_, _, Location, Quaternion],
  Location = [LX, LY, LZ],
  Quaternion = [QX, QY, QZ, QW],
  split_tags(TagsData, Tags),
  ModelProperties = [Id, Raw, LX, LY, LZ, QX, QY, QZ, QW, Tags, Mobility, Gravity, OverlapEvents, Mass].

split_tags(TagsData, Tags) :-
  split_string(TagsData, ";", "", TmpParts),
  delete(TmpParts, "", TmpParts2),
  delete(TmpParts2, "SemLog", Parts),
  make_tags_of_string_parts(Parts, [], Tags).

make_tags_of_string_parts([], Tmp, Tags) :-
  Tags = Tmp.
make_tags_of_string_parts(Parts, Tmp, Tags) :-
  Parts = [Part|Tail],
  split_string(Part, ",", "", Tag),
  FullTag = ["SemLog"|Tag],
  make_tags_of_string_parts(Tail, [FullTag|Tmp], Tags).


remove_namespace(WithNS, Raw) :-
  split_string(WithNS, "#", '"', New ),
  New = [_,Raw].


spawn_constraints(Map, ConstraintProperties) :-
  map_root_object(Map, Constraint),
  rdf_has(Constraint, rdf:type, Name),
  not(rdf_equal(Name, owl:'NamedIndividual')),
  rdf_equal(Name, knowrob:'Constraint'),
  get_constraint_properties(Constraint, Parent, Child, LinConst, AngConst, Pose),
  get_linear_constraint_properties(LinConst, LinProperties),
  get_angular_constraint_properties(AngConst, AngProperties),
  remove_namespace(Constraint, Id),
  spawn_constraint(Id, Parent, Child, LinProperties, AngProperties, Pose, ConstraintProperties).


spawn_relations(Map, Parent, Child) :-
  map_root_object(Map, ParentNS),
  rdf_has(ParentNS, rdf:type, Name),
  not(rdf_equal(Name, owl:'NamedIndividual')),
  rdf_has(ParentNS, knowrob:child, ChildNS),
  remove_namespace(ParentNS, Parent),
  remove_namespace(ChildNS, Child).


spawn_constraint(Id, Parent, Child, LinProperties, AngProperties, Pose, ConstraintProperties) :-
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
  ConstraintProperties = [Id, ParentId, ChildId,
   LX, LY, LZ, QX, QY, QZ, QW,
   LMX, LMY, LMZ, LinLimit, LinSoftConstraint, LinStiffness, LinDamping,
   S1M, S2M, TM, S1L, S2L, TL,
   AngSwingSoftConstraint, AngSwingStiffness, AngSwingDamping,
   AngTwistSoftConstraint, AngTwistStiffness, AngTwistDamping].


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
