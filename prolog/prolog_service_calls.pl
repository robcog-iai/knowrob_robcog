:- module(prolog_service_calls,
    [
    spawn_models/2,
    spawn_constraint/1,
    attach_model_to_parent/2,
    spawn_semantic_map/4,
    spawn_semantic_map/1,
    highlight/1,
    highlight_everything_that_is/2,
    highlight_device_for_action/3,
    highlight_likely_storage_place/2,
    highlight_handle_of/3,
    get_subclass_in_map/3
    ]).

:- use_foreign_library('libprolog_service_calls.so').
:- consult('./spawn_semantic_map_one_service_call.pl').
:- consult('./highlighting_queries.pl').
:- ros_init.
