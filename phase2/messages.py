import threading
import subprocess
import os
import json

"""
%time 1647407887500008867
field.header.seq 14749
field.header.stamp 1647407887500008867
field.header.frame_id base_scan
field.angle_min 0.0
field.angle_max 6.26573181152
field.angle_increment 0.0174532923847
field.time_increment 2.98699997074e-05
field.scan_time 0.0
field.range_min 0.119999997318
field.range_max 3.5
field.ranges0
field.ranges1
field.ranges2
field.ranges3
field.ranges4
...
field.ranges356
field.ranges357
field.ranges358
field.ranges359
field.intensities0
field.intensities1
field.intensities2
field.intensities3
field.intensities4
...
field.intensities352
field.intensities353
field.intensities354
field.intensities355
field.intensities356
field.intensities357
field.intensities358
field.intensities359







max range 4.17399978638
min range 0.0
max intensity 7619.0
min intensity 0.0

max range 4.19799995422
min range 0.0
max intensity 7310.0
min intensity 0.0

max range 4.15100002289
min range 0.0
max intensity 7520.0
min intensity 0.0

max range 4.18800020218
min range 0.0
max intensity 7494.0
min intensity 0.0

max range 4.16900014877
min range 0.0
max intensity 7673.0
min intensity 0.0

max range 4.17000007629
min range 0.0
max intensity 7335.0
min intensity 0.0

max range 4.19299983978
min range 0.0
max intensity 7336.0
min intensity 0.0

max range 4.14400005341
min range 0.0
max intensity 7352.0
min intensity 0.0

max range 4.19999980927
min range 0.0
max intensity 7540.0
min intensity 0.0

max range 4.17999982834
min range 0.0
max intensity 7526.0
min intensity 0.0
"""

class ScanIn:
    def __init__(self) -> None:
        self.p = subprocess.Popen(
            ["rostopic", "echo", "-p", "/scanx"],
            stdout=subprocess.PIPE
        )
        self.labels = self._readline()
    def _readline(self):
        return self.p.stdout.readline().decode().rstrip().split(",")
    def get(self):
        return dict(zip(self.labels, self._readline()))

scan_num = 0
def scan_out(mapping):
    global scan_num
    cmd = (
        "rostopic pub -1 /scan sensor_msgs/LaserScan "
        """'{
            header: {
                seq: %s,
                frame_id: \"%s\"
            },
            angle_min: %s,
            angle_max: %s,
            angle_increment: %s,
            time_increment: %s,
            scan_time: %s,
            range_min: %s,
            range_max: %s,
            ranges: [%s],
            intensities: [%s]
        }'"""
        % (
            # mapping["field.header.seq"],
            str(scan_num),
            # mapping["field.header.stamp"],
            mapping["field.header.frame_id"],
            mapping["field.angle_min"],
            mapping["field.angle_max"],
            mapping["field.angle_increment"],
            mapping["field.time_increment"],
            mapping["field.scan_time"],
            mapping["field.range_min"],
            mapping["field.range_max"],
            ",".join(
                mapping["field.ranges" + str(k)]
                for k
                in range(360)
            ),
            ",".join(
                mapping["field.intensities" + str(k)]
                for k
                in range(360)
            )
        )
    )
    scan_num += 1
    # print(cmd)
    os.system(cmd)

class LockBox:
    def __init__(self) -> None:
        self.val = None
        self.lock = threading.Lock()
    def get(self):
        with self.lock:
            ret = self.val
        return ret
    def set(self, val):
        with self.lock:
            self.val = val

class Trader:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.gotten = threading.Event()
    def pull(self):
        self.gotten.wait()
        with self.lock:
            ret = self.val
            self.gotten.clear()
        return ret
    def push(self, val):
        with self.lock:
            self.val = val
            self.gotten.set()

def scan_thread(box):
    scanner_in = ScanIn()
    while True:
        box.push(scanner_in.get())

def main():
    box = Trader()
    threading.Thread(target=scan_thread, args=(box, ), daemon=True).start()

    # /home/liam/anaconda3/envs/img_getter/bin/python
    getter = subprocess.Popen(
        ["sh", "-c", "unset PYTHONPATH && /home/liam/anaconda3/envs/img_getter/bin/python img_getter.py 192.168.1.185"],
        stdout=subprocess.PIPE
    )

    while True:
        qr_seen = getter.stdout.read(1)
        print(qr_seen)
        mapping = box.pull()
        if qr_seen == b"1":
            for ang in range(160, 200 + 1):
                ang = (ang + 180) % 360
                ang = str(ang)
                mapping["field.ranges" + ang] = "0.5"
                mapping["field.intensities" + ang] = "7500.0"
        scan_out(mapping)

if __name__ == "__main__":
    main()