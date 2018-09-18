:- module(display_types_service_calls,
    [
    display_basic_marker/5,
    display_trajectory_marker/4,
    remove_marker/1
    ]).

:- use_foreign_library('libdisplay_types_service_calls.so').
:- ros_init.
