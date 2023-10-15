


class Test1:
    def __init__(self) -> None:
        self.x = 0
        self.flag = False
        

    def input_x(self):
        while True:
            self.x = float(input(f"Enter the x-coordinate for point : "))
            self.flag =True
            
            