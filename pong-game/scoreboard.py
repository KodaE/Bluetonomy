from turtle import Turtle


class Scoreboard(Turtle):
    def __init__(self):
        super().__init__()
        self.hideturtle()
        self.goto(0, 200)
        self.score_l = 0
        self.score_r = 0
        self.color("white")
        self.score_display()

    def add_score_left(self):
        self.score_l += 1
        self.clear()
        self.score_display()

    def add_score_right(self):
        self.score_r += 1
        self.clear()
        self.score_display()

    def score_display(self):
        self.write(f"{self.score_l} : {self.score_r}", align="center", font=("Courier", 70, "bold"))




