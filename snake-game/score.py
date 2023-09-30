from turtle import Turtle
ALIGNMENT = "center"
FONT = ("Courier", 20, "bold")


class Score(Turtle):
    def __init__(self):
        super().__init__()
        self.score = 0
        with open('data.txt') as data:
            self.high_score = int(data.read())
        self.hideturtle()
        self.color("white")
        self.penup()
        self.goto(0, 280)
        self.update_score()

    def update_score(self):
        self.clear()
        self.write(f"Score: {self.score} High Score: {self.high_score}", align=ALIGNMENT, font=FONT)

    def reset_score(self):
        if self.score > self.high_score:
            self.high_score = self.score
            with open("data.txt", mode="w") as data:
                data.write(f"{self.high_score}")

        self.score = 0
        self.update_score()

    # def game_over(self):
    #     self.goto(0,0)
    #     self.write(f"GAME OVER", align=ALIGNMENT, font=FONT)

    def score_add(self):
        self.score += 1
        self.update_score()
