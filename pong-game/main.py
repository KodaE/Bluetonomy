from turtle import Turtle, Screen
from paddle import Paddle
from ball import Ball
import time as t
from scoreboard import Scoreboard


game_screen = Screen()
game_screen.tracer(0)
game_screen.bgcolor("black")
game_screen.setup(width=800, height=600)
game_screen.title("My Pong Game")
paddle_r = Paddle((350, 0))
paddle_l = Paddle((-350, 0))
ball = Ball()
scoreboard = Scoreboard()



game_screen.update()

game_screen.listen()
game_screen.onkey(key="Up", fun=paddle_r.up)
game_screen.onkey(key="Down", fun=paddle_r.down)
game_screen.onkey(key="w", fun=paddle_l.up)
game_screen.onkey(key="s", fun=paddle_l.down)


game_is_on = True

while game_is_on:
    game_screen.update()
    t.sleep(ball.movement_speed)
    ball.move()
    # Detect wall collision
    if ball.ycor() > 280 or ball.ycor() < -280:
        ball.bounce_y()
    # Detect paddle collision
    if (ball.distance(paddle_r) < 50 and ball.xcor() > 320) or (ball.distance(paddle_l) < 50 and ball.xcor() < -320):
        ball.bounce_x()
    # Detect when players' paddle misses the ball

    # right paddle misses the ball
    if ball.distance(paddle_r) > 50 and ball.xcor() > 380:
        ball.reset_position()
        scoreboard.add_score_left()

    # left paddle misses the ball
    if ball.distance(paddle_l) > 50 and ball.xcor() < -380:
        ball.reset_position()
        scoreboard.add_score_right()






game_screen.exitonclick()