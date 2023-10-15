import Reach_Interface as ri

class Test2:
    def __init__(self) -> None:
        
        pass

    def output_x(self):
        while True:
            if ri.Reach_Int_Class().flag:
                print(ri.Reach_Int_Class().x)
                ri.Reach_Int_Class().flag = False