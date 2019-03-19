identical_maps(Map1, Map2):-
  identical_maps(Map1, Map2, 0).

identical_maps(Map1, Map2, NumericalDelta):-
    map_instance(Map1),
    map_instance(Map2),
    Map1 \= Map2,
    identical_individuals(Map1, Map2, NumericalDelta).


identical_individuals(Map1, Map2, NumericalDelta) :-
  findall(Map1IndividualId,  map_root_object(Map1, Map1IndividualId), Map1Ids),
  findall(Map2IndividualId,  map_root_object(Map2, Map2IndividualId), Map2Ids),
  sort(Map1Ids, Map1IdsSorted),
  sort(Map2Ids, Map2IdsSorted),
  Map1IdsSorted = Map2IdsSorted,
  partition(is_constraint, Map1Ids, Constraints, Objects),
  identical_constraints(Constraints, NumericalDelta),
  identical_objects(Objects, NumericalDelta).

debug_failed_items(Map1, Map2, NumericalDelta, CS, CF, OS, OF) :-
  map_instance(Map1),
  map_instance(Map2),
  Map1 \= Map2,
  findall(Map1IndividualId,  map_root_object(Map1, Map1IndividualId), Map1Ids),
  findall(Map2IndividualId,  map_root_object(Map2, Map2IndividualId), Map2Ids),
  sort(Map1Ids, Map1IdsSorted),
  sort(Map2Ids, Map2IdsSorted),
  Map1IdsSorted = Map2IdsSorted,
  partition(is_constraint, Map1Ids, Constraints, Objects),
  identical_constraints_debug(Constraints, NumericalDelta, CS, CF),
  identical_objects_debug(Objects, NumericalDelta, OS, OF).

identical_objects(Objects, NumericalDelta):-
  maplist(identical_object(NumericalDelta), Objects).

identical_constraints(Constraints, NumericalDelta):-
  maplist(identical_constraint(NumericalDelta), Constraints).


identical_objects_debug(Objects, NumericalDelta, Succeded, Failed):-
  partition(identical_object(NumericalDelta), Objects, Succeded, Failed).

identical_constraints_debug(Constraints, NumericalDelta, Succeded, Failed):-
  partition(identical_constraint(NumericalDelta), Constraints, Succeded, Failed).

identical_constraint(NumericalDelta, Constraint) :-
  findall(Parent, rdf_has(Constraint, knowrob:parent, Parent) , Parents),
  length(Parents, I),  I<2,
  findall(Child, rdf_has(Constraint, knowrob:child, Child) , Children),
  length(Children, I),  I<2,
  findall(TagsData, tags_sorted(Constraint, TagsData), TagsDatasList),
  list_to_set(TagsDatasList, TagDataSet),
  length(TagDataSet, 1),
  identical_poses(NumericalDelta, Constraint),
  identical_linear_constraint(NumericalDelta, Constraint),
  identical_angular_constraint(NumericalDelta, Constraint).



identical_linear_constraint(NumericalDelta, Constraint) :-
  findall(LinConst, rdf_has(Constraint, knowrob:linearConstraint, LinConst) , [Id1, Id2 | _]),
  rdf_has(Id1, knowrob:xMotion, XM),
  rdf_has(Id2, knowrob:xMotion, XM),
  rdf_has(Id1, knowrob:yMotion, YM),
  rdf_has(Id2, knowrob:yMotion, YM),
  rdf_has(Id1, knowrob:zMotion, ZM),
  rdf_has(Id2, knowrob:zMotion, ZM),
  rdf_has(Id1, knowrob:limit, literal(type(xsd:float, Limit1))),
  rdf_has(Id2, knowrob:limit, literal(type(xsd:float, Limit2))),
  rdf_has(Id1, knowrob:softConstraint, SoftConst),
  rdf_has(Id2, knowrob:softConstraint, SoftConst),
  rdf_has(Id1, knowrob:stiffness, literal(type(xsd:float, Stiffness1))),
  rdf_has(Id2, knowrob:stiffness, literal(type(xsd:float, Stiffness2))),
  rdf_has(Id1, knowrob:damping, literal(type(xsd:float, Damping1))),
  rdf_has(Id2, knowrob:damping, literal(type(xsd:float, Damping2))),
  foldl(makeAtomToNumAndAddToList, [Limit1, Stiffness1, Damping1], [], Nums1),
  foldl(makeAtomToNumAndAddToList, [Limit2, Stiffness2, Damping2], [], Nums2),
  maplist(compare_with_delta(NumericalDelta), Nums1, Nums2).



identical_angular_constraint(NumericalDelta, Constraint) :-
  findall(AngConst, rdf_has(Constraint, knowrob:angularConstraint, AngConst) , [Id1, Id2 | _]),
  rdf_has(Id1, knowrob:swing1Motion, Swing1M),
  rdf_has(Id2, knowrob:swing1Motion, Swing1M),
  rdf_has(Id1, knowrob:swing2Motion, Swing2M),
  rdf_has(Id2, knowrob:swing2Motion, Swing2M),
  rdf_has(Id1, knowrob:twistMotion, TwistM),
  rdf_has(Id2, knowrob:twistMotion, TwistM),

  rdf_has(Id1, knowrob:swing1Limit, literal(type(xsd:float, Swing1L1))),
  rdf_has(Id2, knowrob:swing1Limit, literal(type(xsd:float, Swing1L2))),
  rdf_has(Id1, knowrob:swing2Limit, literal(type(xsd:float, Swing2L1))),
  rdf_has(Id2, knowrob:swing2Limit, literal(type(xsd:float, Swing2L2))),
  rdf_has(Id1, knowrob:twistLimit, literal(type(xsd:float, TwistL1))),
  rdf_has(Id2, knowrob:twistLimit, literal(type(xsd:float, TwistL2))),

  rdf_has(Id1, knowrob:softSwingConstraint, SSC),
  rdf_has(Id2, knowrob:softSwingConstraint, SSC),
  rdf_has(Id1, knowrob:swingStiffness, literal(type(xsd:float, SS1))),
  rdf_has(Id2, knowrob:swingStiffness, literal(type(xsd:float, SS2))),
  rdf_has(Id1, knowrob:swingDamping, literal(type(xsd:float, SD1))),
  rdf_has(Id2, knowrob:swingDamping, literal(type(xsd:float, SD2))),

  rdf_has(Id1, knowrob:softTwistConstraint, STC),
  rdf_has(Id2, knowrob:softTwistConstraint, STC),
  rdf_has(Id1, knowrob:twistStiffness, literal(type(xsd:float, TS1))),
  rdf_has(Id2, knowrob:twistStiffness, literal(type(xsd:float, TS2))),
  rdf_has(Id1, knowrob:twistDamping, literal(type(xsd:float, TD1))),
  rdf_has(Id2, knowrob:twistDamping, literal(type(xsd:float, TD2))),

  foldl(makeAtomToNumAndAddToList, [Swing1L1, Swing2L1, TwistL1, SS1, SD1, TS1, TD1], [], Nums1),
  foldl(makeAtomToNumAndAddToList, [Swing1L2, Swing2L2, TwistL2, SS2, SD2, TS2, TD2], [], Nums2),
  maplist(compare_with_delta(NumericalDelta), Nums1, Nums2).



identical_object(NumericalDelta, Obj) :-
  findall(Parent, rdf_has(Obj, knowrob:parent, Parent) , Parents),
  length(Parents, I),  I<2,
  findall(Mobility, rdf_has(Obj, knowrob:mobility, Mobility) , Mobilitys),
  length(Mobilitys, 1),
  findall(Mass, rdf_has(Obj, knowrob:mass, Mass) , Masses),
  length(Masses, 1),
  findall(OverlapEvent, rdf_has(Obj, knowrob:overlapEvents, OverlapEvent) , OverlapEvents),
  length(OverlapEvents, 1),
  findall(Gravity, rdf_has(Obj, knowrob:gravity, Gravity) , Gravitys),
  length(Gravitys, 1),
  findall(PathToCadModel, rdf_has(Obj, knowrob:overlapEvents, PathToCadModel) , PathsToCadModel),
  length(PathsToCadModel, 1),
  findall(TagsData, tags_sorted(Obj, TagsData), TagsDatasList),
  list_to_set(TagsDatasList, TagDataSet),
  length(TagDataSet, 1),
  identical_poses(NumericalDelta, Obj).

tags_sorted(Obj, TagsData) :-
  rdf_has(Obj, knowrob:tagsData,  literal(type(xsd:string, DataString))),
  string_to_list(DataString, L), sort(L, TagsData).


compare_with_delta(D, N1, N2) :-
 H is N1 + D,
 L is N1 - D,
 L =< N2, N2 =< H.


identical_poses(NumericalDelta, Obj) :-
  findall(Pose, rdf_has(Obj, knowrob:pose, Pose), [Id1, Id2 | _]),
  rdf_has(Id1, knowrob:quaternion, literal(type(xsd:string, Quat1String))),
  rdf_has(Id2, knowrob:quaternion, literal(type(xsd:string, Quat2String))),
  rdf_has(Id1, knowrob:translation, literal(type(xsd:string, Trans1String))),
  rdf_has(Id2, knowrob:translation, literal(type(xsd:string, Trans2String))),
  nums_of_string(Quat1String, Quat1),
  nums_of_string(Quat2String, Quat2),
  nums_of_string(Trans1String, Trans1),
  nums_of_string(Trans2String, Trans2),
  (maplist(compare_with_delta(NumericalDelta), Quat1, Quat2 );
  foldl(negate_and_add_to_list, Quat1, [], NegQuat1 ), maplist(compare_with_delta(NumericalDelta), NegQuat1, Quat2)),
  maplist(compare_with_delta(NumericalDelta), Trans1, Trans2).


nums_of_string(String, Nums) :-
  split_string(String, " ", "", NumStrings),
  foldl(makeNumAndAddToList, NumStrings, [], Nums).

makeNumAndAddToList(NumStr, Tmp, List) :-
  number_string(Num, NumStr),
  List = [Num | Tmp].

makeAtomToNumAndAddToList(NumAtom, Tmp, List) :-
  atom_number(NumAtom, Num),
  List = [Num | Tmp].

negate_and_add_to_list(Num, Tmp, List) :-
  Neg is Num * -1,
  append(Tmp, [Neg], List).


analyse_semantic_map(Map, NumberOfDifferentClassTypes,  NumberOfIndividuals, NumberOfObjects, NumberOfConstraints, NumberOfRelations):-
  map_instance(Map),
  findall(IndividualId,  map_root_object(Map, IndividualId), IndividualIds),
  length(IndividualIds, NumberOfIndividuals),
  get_list_of_classes(IndividualIds, Classes),
  list_to_set(Classes, ClassesUnique),
  length(ClassesUnique, NumberOfDifferentClasses),
  partition(is_constraint, IndividualIds, Constraints, Objects),
  length(Objects, NumberOfObjects),
  length(Constraints, NumberOfConstraints),
  partition(is_child, IndividualIds, Children, _),
  length(Children, NumberOfRelations).


is_child(Id):-
  rdf_has(Id, knowrob:parent, _).


is_constraint(Id):-
  rdf_has(Id, rdf:type, Type),
  rdf_equal(Type,  knowrob:'Constraint').


get_list_of_classes(IndividualIds, Classes):-
  get_list_of_classes(IndividualIds, Classes, []).

get_list_of_classes([], Classes, Tmp):-
  Classes = Tmp.

get_list_of_classes(IndividualIds, Classes, Tmp) :-
  IndividualIds = [Id | Tail],
  rdf_has(Id, rdf:type, Name),
  not(rdf_equal(Name, owl:'NamedIndividual')),
  get_list_of_classes(Tail, Classes, [Name | Tmp]).
