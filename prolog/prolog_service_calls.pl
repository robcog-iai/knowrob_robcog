:- module(prolog_service_calls,
    [
    spawn_model/2,
    spawn_constraint/1,
    attach_model_to_parent/2,
    spawn_semantic_map/4
    ]).

:- use_foreign_library('libprolog_service_calls.so').
:- consult('./spawn_semantic_map.pl').
:- ros_init.
