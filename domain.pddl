(define (domain bookWorld)

    (:requirements
        :equality
        :typing
        :strips
    )

    (:types
        robot
        book
        bin
        location
        subject
        size
    )


    (:predicates
        (Book_At ?a - book ?b - location)
        (Bin_At ?a - bin ?b - location)
        (Book_Subject ?a - book ?b - subject)
        (Book_Size ?a - book ?b - size)
        (Bin_Subject ?a - bin ?b - subject)
        (Bin_Size ?a - bin ?b - size)
        (Robot_At ?a - robot ?b - location)
        (Empty_Basket ?a - robot)
        (In_Basket ?b - book)
    )


    (:action pick
        :parameters (?book - book ?location - location ?robot -robot)
        :precondition (
            and (Book_At ?book ?location) (Robot_At ?robot ?location) (Empty_Basket ?robot)
        )
        :effect ( 
            and (not (Book_At ?book ?location)) (not (Empty_Basket ?robot)) (In_Basket ?book)
        )
    )

    (:action place
        :parameters (?book - book ?bin - bin ?subject - subject ?size - size ?location - location ?robot - robot)
        :precondition (
            and (Robot_At ?robot ?location) (Bin_At ?bin ?location) (In_Basket ?book) (Bin_Size ?bin ?size) (Book_Size ?book ?size) (Bin_Subject ?bin ?subject) (Book_Subject ?book ?subject) 
		)
        :effect (
            and (not (In_Basket ?book)) (Empty_Basket ?robot) (Book_At ?book ?location) 
        )
    )
    
    (:action move
        :parameters (?robot - robot ?from_location ?to_location - location)
        :precondition (
            Robot_At ?robot ?from_location
        )
        :effect (
            and (Robot_At ?robot ?to_location) (not (Robot_At ?robot ?from_location)) 
        )
    )
)

