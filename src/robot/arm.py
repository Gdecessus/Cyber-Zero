import os
import json
import time
import requests
from urllib.parse import quote

ARM_URL = "http://192.168.4.1"
HAND_OPEN = 2.78
HAND_CLOSED = 2.99
POSES_PATH = os.path.join(os.path.dirname(__file__), "poses.json")


class Arm:

    def __init__(self, poses_path=POSES_PATH):
        self.session = requests.Session()
        self.poses = {}

        if os.path.exists(poses_path):
            with open(poses_path, "r") as f:
                self.poses = json.load(f)
            print(f"Loaded {len(self.poses)} poses")
        else:
            print(f"No poses file at {poses_path}")

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

    def pick(self, square):
        square = square.upper()
        travel = self.poses[square]["hover"]
        approach = self.poses[square]["approach"]
        descend = self.poses[square]["descend"]

        self.move_to(travel, HAND_OPEN)
        time.sleep(1.5)

        self.move_to(approach, HAND_OPEN)
        time.sleep(0.8)
        self.settle_shoulder(approach, HAND_OPEN)

        self.move_to(descend, HAND_OPEN)
        time.sleep(1.0)
        self.settle_elbow(descend, HAND_OPEN)

        self.move_to(descend, HAND_CLOSED)
        time.sleep(0.6)

        self.move_to(approach, HAND_CLOSED)
        time.sleep(0.8)

        self.move_to(travel, HAND_CLOSED)
        time.sleep(1.5)

    def place(self, square):
        square = square.upper()
        travel = self.poses[square]["hover"]
        approach = self.poses[square]["approach"]
        drop = self.poses[square]["drop"]

        self.move_to(travel, HAND_CLOSED)
        time.sleep(1.5)

        self.move_to(approach, HAND_CLOSED)
        time.sleep(0.8)
        self.settle_shoulder(approach, HAND_CLOSED)

        self.move_to(drop, HAND_CLOSED)
        time.sleep(1.0)
        self.settle_elbow(drop, HAND_CLOSED)

        time.sleep(0.35)
        self.move_to(drop, HAND_OPEN)
        time.sleep(0.6)

        self.move_to(approach, HAND_OPEN)
        time.sleep(0.8)

        self.move_to(travel, HAND_OPEN)
        time.sleep(1.5)

    def pick_and_place(self, from_sq, to_sq):
        print(f"Moving: {from_sq.upper()} -> {to_sq.upper()}")
        self.pick(from_sq)
        self.place(to_sq)
        print(f"Done: {from_sq.upper()} -> {to_sq.upper()}")
