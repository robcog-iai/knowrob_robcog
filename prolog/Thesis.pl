author("This Thesis", "Björn Veit").
supervisor("This Thesis", "Andrei Haidu").
examiner("This Thesis", 1, "Prof. Michael Beetz PhD").
examiner("This Thesis", 2, "Robert Porzel").

examiner(Publication, Examiner) :-
examiner(Publication, _, Examiner).

work_together(A, B) :-
    author(Publication, A), supervisor(Publication, B);
    author(Publication, B), supervisor(Publication, A);
    examiner(Publication, A),  examiner(Publication, B), A \== B.



