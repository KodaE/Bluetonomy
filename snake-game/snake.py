from turtle import Turtle

STARTING_POS = [(0, 0), (-20, 0), (-40, 0)]
MOVE_DISTANCE = 20
UP = 90
DOWN = 270
RIGHT = 0
LEFT = 180


class Snake:
    def __init__(self):
        self.snake_list = []
        self.create_snake()
        self.head = self.snake_list[0]

    def extend_snake(self):
        self.add_snake(self.snake_list[-1].position())

    def add_snake(self, position):
        snake = Turtle(shape="square")
        snake.penup()
        snake.color("white")
        snake.goto(position)
        self.snake_list.append(snake)

    def reset(self):
        for pieces in self.snake_list:
            pieces.hideturtle()
        self.snake_list.clear()
        self.create_snake()
        self.head = self.snake_list[0]

    def create_snake(self):
        for position in STARTING_POS:
            self.add_snake(position)

    def move(self):
        for snake_num in range(len(self.snake_list) - 1, 0, -1):
            new_x = self.snake_list[snake_num - 1].xcor()
            new_y = self.snake_list[snake_num - 1].ycor()
            self.snake_list[snake_num].goto(new_x, new_y)

        self.head.forward(MOVE_DISTANCE)

    def down(self):
        if self.head.heading() != UP:
            self.head.setheading(DOWN)

    def up(self):
        if self.head.heading() != DOWN:
            self.head.setheading(UP)

    def left(self):
        if self.head.heading() != RIGHT:
            self.head.setheading(LEFT)

    def right(self):
        if self.head.heading() != LEFT:
            self.head.setheading(RIGHT)
