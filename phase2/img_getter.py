from multiprocessing.connection import Client
import sys
import cv2
import numpy as np
from itertools import count, cycle, islice
from math import fabs, floor
import json
import os

def pairwise(it):
    try:
        a = next(it)
        while True:
            b = next(it)
            yield (a, b)
            a = b
    except StopIteration:
        pass

qr_decoder = cv2.QRCodeDetector()

with Client((sys.argv[1], 3100)) as cli:
    for i in count():
        cli.send(None)
        img_bytearray = cli.recv()

        img_flat = np.frombuffer(img_bytearray, np.uint8)
        img = cv2.imdecode(img_flat, cv2.IMREAD_COLOR)

        found, points_groups = qr_decoder.detect(img)
        if found:
            sys.stdout.write("1")
            sys.stdout.flush()
            for points in points_groups:
                for line in islice(pairwise(cycle(points)), 4):
                    # print(line[0])
                    # print(line[1])
                    cv2.line(
                        img,
                        tuple(map(floor, line[0])),
                        tuple(map(floor, line[1])),
                        (0, 255, 0),
                        thickness=2
                    )
        else:
            sys.stdout.write("0")
            sys.stdout.flush()

        cv2.imshow("raspistill stream", img)
        cv2.waitKey(1)