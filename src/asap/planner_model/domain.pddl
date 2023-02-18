(define (domain asap_domain)

    (:requirements :strips :typing :adl :fluents :numeric-fluents :durative-actions :timed-initial-fluents :timed-initial-literals :time )

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:types
        table
        waypoint
        robot
    )

    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates

        (robot_at ?roboter - robot ?wp - waypoint)
    	(robot_attable ?roboter - robot ?table - table)
        (free ?roboter - robot)
        (not_free ?roboter - robot)
        (visited ?wp - waypoint)
    )

    ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:functions
        (n_drinks ?table - table) - number
        (distance ?wpa ?wpb - waypoint)
    )

    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; goto_waypoint
    ; Roboter "roboter" bewegt sich von Waypoint "from" zu Waypoint "to"
    (:durative-action goto_waypoint
        :parameters (
            ?roboter - robot 
            ?from - waypoint
            ?to - waypoint
        )
        :duration ( = ?duration (distance ?from ?to))
        :condition (and
            (at start (robot_at ?roboter ?from)))
        :effect (and
            (at end (visited ?to))
            (at start (not (robot_at ?roboter ?from)))
            (at end (robot_at ?roboter ?to)))
    )

    ;; moveto_table
    ; Roboter "roboter" bewegt sich von Location "fromloc" zu Location "toloc"
    (:durative-action moveto_table
        :parameters (
            ?roboter - robot 
            ?from - table 
            ?to - table
        )
        :duration (= ?duration 1)
        :condition (at start (and 
            (robot_attable ?roboter ?from)
        ))
        :effect (and 
            (at end (robot_attable ?roboter ?to))
            (at start (not  (robot_attable ?roboter ?from)))
        )
    )
    
    ;; pickup
    ; Roboter "roboter" hebt Getränk vom Tisch "table" auf.
    (:durative-action pickup
        :parameters  ( 
            ?roboter - robot 
            ?table - table
        )
        :duration (= ?duration 1)
        :condition (at start (and 
            (robot_attable ?roboter ?table)
            (> (n_drinks ?table) 0)
            (free ?roboter))
        )
        :effect (and 
            (at end (and 
                (decrease (n_drinks ?table) 1)
                (not_free ?roboter))
            )
            (at start (not (free ?roboter)))
        )
    )

    ;; putdown
    ;Roboter "roboter" legt Getränk am Tisch "table" ab.
    (:durative-action putdown
        :parameters ( 
            ?roboter - robot 
            ?table - table 
        )
        :duration (= ?duration 1)
        :condition (at start (and 
            (robot_attable ?roboter ?table)
            (< (n_drinks ?table) 2)
            (not_free ?roboter))
        )
        :effect (and 
            (at end (and 
                (increase (n_drinks ?table) 1)
                (free ?roboter))
            )
            (at start (not (not_free ?roboter)))
        )
    )

    ;; end Actions ;;;;;;;;;;;;;;;;;;;;;;;;

)
