from turtle import Screen
import time as t
from snake import Snake
from food import Food
from score import Score

# Variables initialization


# Screen set up
my_screen = Screen()
my_screen.tracer(0)
my_screen.setup(width=600, height=600)
my_screen.bgcolor("black")
my_screen.title("My snake game")

snake = Snake()
food = Food()
score = Score()


# # Create the snake
# snake = Turtle("square")
# snake.color("white")
# snake_head_pos = snake.pos()
# snake_1 = Turtle("square")
# snake_1.color("white")
# snake_1.goto(x= snake_head_pos[0]-20, y=snake_head_pos[1])


my_screen.listen()
my_screen.onkey(key="Up", fun=snake.up)
my_screen.onkey(key="Down", fun=snake.down)
my_screen.onkey(key="Right", fun=snake.right)
my_screen.onkey(key="Left", fun=snake.left)

is_game_on = True

while is_game_on:

    my_screen.update()
    t.sleep(0.1)
    snake.move()

    # Detect collision with food
    if snake.head.distance(food) < 15:
        food.refresh()
        score.score_add()
        snake.extend_snake()

    # Detect collision with screen border
    if snake.head.xcor() > 280 or snake.head.xcor() < -280 or snake.head.ycor() > 280 or snake.head.ycor() < -280:
        score.reset_score()
        snake.reset()

    # Detect collision with tail
    for snake_body in snake.snake_list[1:]:
        if snake.head.distance(snake_body) < 10:
            score.reset_score()
            snake.reset()


my_screen.exitonclick()
