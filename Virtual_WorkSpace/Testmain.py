from Test1 import Test1
from Test2 import Test2
import threading

t1 = Test1()
t2 = Test2()

test1thread = threading.Thread(target=t1.input_x)
test2thread = threading.Thread(target=t2.output_x)

test1thread.start()
test2thread.start()