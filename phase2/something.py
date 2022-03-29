from multiprocessing.connection import Listener
import os
import uuid

class Raspistill:
    def __init__(self):
        self.tmpfifo = "/tmp/{}".format(uuid.uuid1())
        assert os.system("mkfifo {}".format(self.tmpfifo)) == 0
        self.w = os.popen(
            "raspistill -o - -k -n -e jpg -w 640 -h 480 > {}".format(self.tmpfifo),
            "w"
        )
        self.r = os.open(self.tmpfifo, os.O_RDONLY)
    
    def read(self):
        self.w.write("\n")
        self.w.flush()
        buff = bytearray()
        while len(buff) < 2 or not (buff[-2] == 0xff and buff[-1] == 0xd9):
            buff.extend(os.read(self.r, 10 << 20))
        return buff
    
    def close(self):
        self.w.write("X\n")
        self.w.flush()
        self.w.close()
        os.close(self.r)
        os.remove(self.tmpfifo)

rs = Raspistill()

with Listener("localhost:3100") as l:
    print("listening...")
    with l.accept() as con:
        con.send(rs.read())