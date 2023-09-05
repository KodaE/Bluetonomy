import turtle
from turtle import Turtle
UP = 90
DOWN = 270
turtle.seth(UP)

class Paddle(Turtle):
    def __init__(self, position):
        super().__init__()
        self.shape("square")
        self.color("white")
        self.penup()
        self.setheading(UP)
        self.setposition(position)
        self.turtlesize(stretch_wid=1, stretch_len=5)

    def up(self):
        if self.ycor() >= 250:
            pass
        else:
            self.forward(30)

    def down(self):
        if self.ycor() <= -250:
            pass
        else:
            self.backward(30)



