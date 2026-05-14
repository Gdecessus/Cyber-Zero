import os
import json
import time
import requests
from urllib.parse import quote

ARM_URL = "http://192.168.4.1"

# gripper servo positions — wider value = open hand, narrower = closed
HAND_OPEN = 2.78
HAND_CLOSED = 2.99

POSES_PATH = os.path.join(os.path.dirname(__file__), "poses.json")


class Arm:

    def __init__(self, poses_path=POSES_PATH):
        self.session = requests.Session()
        self.poses = {}

        # load the 64-square joint angles I calibrated by hand
        if os.path.exists(poses_path):
            with open(poses_path, "r") as f:
                self.poses = json.load(f)
            print(f"Loaded {len(self.poses)} poses")
        else:
            print(f"No poses file at {poses_path}")

    # talk to the arm — pack the command into a URL and send it to the arm's web endpoint
    def send(self, cmd):
        cmd_str = json.dumps(cmd, separators=(",", ":"))
        url = f"{ARM_URL}/js?json={quote(cmd_str, safe='')}"
        try:
            r = self.session.get(url, timeout=3.0)
            r.raise_for_status()
            return r.text.strip()
        except requests.exceptions.RequestException as e:
            print(f"Arm error: {e}")
            return None

    # T:1051 is the Waveshare command code that asks the arm to report its joint angles
    def query_state(self):
        resp = self.send({"T": 1051})
        if resp is None:
            return None
        data = json.loads(resp)
        return {
            "base": data["b"],
            "shoulder": data["s"],
            "elbow": data["e"],
            "wrist": data["t"],
            "roll": data["r"],
            "hand": data["g"],
        }

    # T:102 tells the arm to move to a specific set of joint angles
    def move_to(self, pose, hand, spd=0, acc=10):
        cmd = {
            "T": 102,
            "base": pose["base"],
            "shoulder": pose["shoulder"],
            "elbow": pose["elbow"],
            "wrist": pose["wrist"],
            "roll": pose["roll"],
            "hand": hand,
            "spd": spd,
            "acc": acc,
        }
        self.send(cmd)

    # tiny jitter on the shoulder joint — nudge up then down to make sure the
    # piece is properly aligned in the gripper before we commit to lifting
    def settle_shoulder(self, pose, hand, delta=0.02, hold=0.12):
        p_up = dict(pose)
        p_up["shoulder"] = pose["shoulder"] + delta

        p_dn = dict(pose)
        p_dn["shoulder"] = pose["shoulder"] - delta

        self.move_to(p_up, hand)
        time.sleep(hold)
        self.move_to(p_dn, hand)
        time.sleep(hold)
        self.move_to(pose, hand)
        time.sleep(hold)

    # same idea as settle_shoulder but jiggling the elbow — used at descend
    # height to settle the gripper down on the piece without slipping
    def settle_elbow(self, pose, hand, delta=0.02, hold=0.12):
        p_up = dict(pose)
        p_up["elbow"] = pose["elbow"] + delta

        p_dn = dict(pose)
        p_dn["elbow"] = pose["elbow"] - delta

        self.move_to(p_up, hand)
        time.sleep(hold)
        self.move_to(p_dn, hand)
        time.sleep(hold)
        self.move_to(pose, hand)
        time.sleep(hold)

    # full pickup sequence — hover, approach, come down, grip, come back up
    def pick(self, square):
        square = square.upper()
        travel = self.poses[square]["hover"]
        approach = self.poses[square]["approach"]
        descend = self.poses[square]["descend"]

        # hover above the square first so we don't bump anything on the way in
        self.move_to(travel, HAND_OPEN)
        time.sleep(1.5)

        # slow approach down to just above the piece
        self.move_to(approach, HAND_OPEN)
        time.sleep(0.8)
        self.settle_shoulder(approach, HAND_OPEN)

        # all the way down to grip height
        self.move_to(descend, HAND_OPEN)
        time.sleep(1.0)
        self.settle_elbow(descend, HAND_OPEN)

        # close the gripper around the piece
        self.move_to(descend, HAND_CLOSED)
        time.sleep(0.6)

        # lift back up the same way we came in
        self.move_to(approach, HAND_CLOSED)
        time.sleep(0.8)

        self.move_to(travel, HAND_CLOSED)
        time.sleep(1.5)

    # full place sequence — reverse of pick, but with HAND_CLOSED until the drop
    def place(self, square):
        square = square.upper()
        travel = self.poses[square]["hover"]
        approach = self.poses[square]["approach"]
        drop = self.poses[square]["drop"]

        # carry the piece in over the destination
        self.move_to(travel, HAND_CLOSED)
        time.sleep(1.5)

        self.move_to(approach, HAND_CLOSED)
        time.sleep(0.8)
        self.settle_shoulder(approach, HAND_CLOSED)

        # come down to drop height
        self.move_to(drop, HAND_CLOSED)
        time.sleep(1.0)
        self.settle_elbow(drop, HAND_CLOSED)

        # release the piece
        time.sleep(0.35)
        self.move_to(drop, HAND_OPEN)
        time.sleep(0.6)

        # back out the way we came
        self.move_to(approach, HAND_OPEN)
        time.sleep(0.8)

        self.move_to(travel, HAND_OPEN)
        time.sleep(1.5)

    # one chess move = pick from one square, place on another
    def pick_and_place(self, from_sq, to_sq):
        print(f"Moving: {from_sq.upper()} -> {to_sq.upper()}")
        self.pick(from_sq)
        self.place(to_sq)
        print(f"Done: {from_sq.upper()} -> {to_sq.upper()}")
