import Reach_Interface as ri


class Test1:
    def __init__(self) -> None:
        self.s = ri.Reach_Int_Class()
        self.x = 0
        self.flag = False
        

    def input_x(self):
        while True:
            ri.Reach_Int_Class().x = float(input(f"Enter the x-coordinate for point : "))
            ri.Reach_Int_Class().flag = True
            
            