highlight_everything_that_is(Map, Type) :-
  findall(Individual, get_subclass_in_map(Map, Type, Individual), Individuals),
  highlight_all(Individuals).

highlight_device_for_action(Map, Action, Device):-
  owl_class_properties(Action, knowrob:'deviceUsed', Device),
  highlight_everything_that_is(Map, Device).

highlight_likely_storage_place(Map, Type):-
  map_instance(Map),
  likely_storage_place_in_environment(Map, Type, StoragePlace),
  highlight(StoragePlace).

highlight_handle_of(Map, Individual, Handle) :-
  handle_of(Map, Individual, Handle),
  highlight(Handle).

highlight(Individual) :-
  remove_namespace(Individual, Id),
  highlight_models([Id]).

highlight_all(List) :-
  foldl(remove_namespace_and_add_to_list, List, [], Ids),
  highlight_models(Ids).

handle_of(Map, Individual, Handle):-
  map_instance(Map),
  get_all_children(Individual, Children),
  member(Handle, Children),
  rdf_has(Handle, rdf:type, HandleType),
  rdfs_subclass_of(HandleType, 'http://knowrob.org/kb/knowrob.owl#Handle').

get_subclass_in_map(Map, Type, Individual) :-
  rdfs_individual_of(Individual, Type),
  map_root_object(Map, Individual).

likely_storage_place_in_environment_with_highlighting(Map, ItemType, StorageIndividual) :-
  likely_storage_place_in_environment(Map, ItemType, StorageIndividual),
  remove_namespace(StorageIndividual, Id),
  highlight_models([Id]).

likely_storage_place_in_environment(Map, ItemType, StorageIndividual) :-
  likely_storage_place(ItemType, StoragePlace),
  map_root_object(Map, StorageIndividual),
  rdf_has(StorageIndividual, rdf:type, StoragePlace).

likely_storage_place(ItemType, StoragePlace) :-
  rdfs_subclass_of(ItemType, Subclass),
  owl_class_properties(StoragePlace, knowrob:'typePrimaryFunction-containerFor', Subclass).

get_all_children(Individual, Children) :-
  get_all_children(Individual, [], Children).

get_all_children(Individual, Tmp, Children) :-
  get_all_direct_children(Individual, DirectChildren),
  (not(length(DirectChildren, 0)) -> foldl(get_all_children, DirectChildren, DirectChildren, GrandChildren); true),
  append(GrandChildren, Tmp, Children).

get_all_direct_children(Individual, Children) :-
  findall(Child, rdf_has(Child, knowrob:parent, Individual), NewKids1),
  findall(Child, rdf_has(Individual, knowrob:child, Child), NewKids2),
  append(NewKids1, NewKids2, NewKidsList),
  list_to_set(NewKidsList,  NewKids),
  Children = NewKids.

is_constraint(Individual):-
  rdf_has(Individual, rdf:type, Type),
  rdf_equal(Type,  knowrob:'Constraint').

remove_namespace_and_add_to_list(WithNs, Tmp, List) :-
  split_string(WithNs, "#", '"', New ),
  New = [_,Raw],
  append(Tmp, [Raw], List).
