(define (problem asap_problem)
(:domain asap_domain)
(:objects
    robot - robot
    tablea1 tablea2 tablea3 tablea4 tableb1 tableb2 tableb3 tableb4 - table
)
(:init
    (robot_attable robot tablea1)
    (= (n_drinks tablea1) 2)
    (= (n_drinks tablea2) 2)
    (= (n_drinks tablea3) 2)
    (= (n_drinks tablea4) 2)
    (= (n_drinks tableb1) 0)
    (= (n_drinks tableb2) 0)
    (= (n_drinks tableb3) 0)
    (= (n_drinks tableb4) 0)
    (free robot)
)
(:goal (and
    (= (n_drinks tableb1) 2)
))
)
