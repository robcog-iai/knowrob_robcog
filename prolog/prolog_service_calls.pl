:- module(prolog_service_calls,
    [
    spawn_model/9,
    spawn_constraint/1,
    attach_model_to_parent/2
    ]).

:- use_foreign_library('libprolog_service_calls.so').
:- ros_init.
