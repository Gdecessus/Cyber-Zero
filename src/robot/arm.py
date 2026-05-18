import os
import json
import time
import requests
from urllib.parse import quote

ARM_URL = "http://192.168.4.1"

# gripper servo positions — LOWER value = wider/more open, HIGHER = more closed
HAND_OPEN = 2.78
HAND_CLOSED = 2.99
# scoop = wider than HAND_OPEN, used only during pickup descent so the gripper
# is wide enough to envelop slightly off-centre pieces. Set HAND_SCOOP = HAND_OPEN
# to disable scoop behaviour (reverts to old single-width pickup).
HAND_SCOOP = 2.65

# The gripper opens only on the A side (the H-side jaw is static). The scoop
# is encoded directly in poses.json: each square's approach.base sits +0.025
# toward H of its descend.base, so the natural descent path (approach -> descend)
# is diagonal H -> A, which acts as the scoop.

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

    # full pickup sequence — hover, approach, descend, grip, come back up.
    # The H-side scoop is encoded directly in poses.json: approach.base sits
    # slightly toward H of descend.base, so the descent from approach to
    # descend is naturally diagonal (H -> A) and acts as the scoop.
    def pick(self, square):
        square = square.upper()
        travel = self.poses[square]["hover"]
        approach = self.poses[square]["approach"]
        descend = self.poses[square]["descend"]

        # hover above the square first so we don't bump anything on the way in
        self.move_to(travel, HAND_SCOOP)
        time.sleep(1.2)

        # approach (H-shifted in poses.json) — gripper just above and toward H
        self.move_to(approach, HAND_SCOOP)
        time.sleep(0.8)

        # descend straight down to the calibrated descend pose; the diagonal
        # path from approach to descend is the scoop
        self.move_to(descend, HAND_SCOOP)
        time.sleep(0.9)

        # close the gripper around the piece
        self.move_to(descend, HAND_CLOSED)
        time.sleep(0.6)

        # lift back up the same way we came in
        self.move_to(approach, HAND_CLOSED)
        time.sleep(0.8)

        self.move_to(travel, HAND_CLOSED)
        time.sleep(1.2)

    # full place sequence — reverse of pick, but with HAND_CLOSED until the drop
    def place(self, square):
        square = square.upper()
        travel = self.poses[square]["hover"]
        approach = self.poses[square]["approach"]
        drop = self.poses[square]["drop"]

        # carry the piece in over the destination
        self.move_to(travel, HAND_CLOSED)
        time.sleep(1.2)

        self.move_to(approach, HAND_CLOSED)
        time.sleep(0.8)

        # come down to drop height
        self.move_to(drop, HAND_CLOSED)
        time.sleep(0.9)

        # release the piece
        self.move_to(drop, HAND_OPEN)
        time.sleep(0.5)

        # back out the way we came
        self.move_to(approach, HAND_OPEN)
        time.sleep(0.8)

        self.move_to(travel, HAND_OPEN)
        time.sleep(1.2)

    # one chess move = pick from one square, place on another
    def pick_and_place(self, from_sq, to_sq):
        print(f"Moving: {from_sq.upper()} -> {to_sq.upper()}")
        self.pick(from_sq)
        self.place(to_sq)
        print(f"Done: {from_sq.upper()} -> {to_sq.upper()}")
