

class Test2:
    def __init__(self,t1) -> None:
        self.t1 = t1
        pass

    def output_x(self):
        while True:
            if self.t1.flag:
                print(self.t1.x)
                self.t1.flag = False