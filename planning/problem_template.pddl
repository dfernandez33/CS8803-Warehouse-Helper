(define (problem pb1)
(:domain warehouse)

(:objects 	ball_shelf beer_shelf cup_shelf cargo_bay starting_location
		    ball_1 ball_2 ball_3
		    beer_1 beer_2 beer_3
		    cup_1 cup_2 cup_3)

(:init 		(in ball_1 ball_shelf)
            (in ball_2 ball_shelf)
            (in ball_3 ball_shelf)
            (in beer_1 beer_shelf)
            (in beer_2 beer_shelf)
            (in beer_3 beer_shelf)
            (in cup_1 cup_shelf)
            (in cup_2 cup_shelf)
            (in cup_3 cup_shelf)
		    (at starting_location))
