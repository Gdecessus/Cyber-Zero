"""
Flask API server for controlling the Waveshare RoArm-M3-S.

Run on the Raspberry Pi:
    python3 -m src.robot.arm_server

Endpoints:
    GET  /status              - query arm joint positions
    GET  /poses               - list all squares
    GET  /poses/<square>      - get poses for a square
    POST /move                - move to square + phase
    POST /pick                - full pick sequence
    POST /place               - full place sequence
    POST /pick_and_place      - pick from -> place to
    POST /adjust              - nudge a joint by delta
    POST /set                 - set exact joint value
    POST /save                - save poses.json
    POST /home                - move to init position
    POST /raw                 - send raw JSON command
"""

import json
from flask import Flask, request, jsonify
from flask_cors import CORS
from src.robot.arm import Arm, POSES_PATH

app = Flask(__name__)
CORS(app)

arm = None


def get_arm():
    global arm
    if arm is None:
        arm = Arm()
    return arm


@app.route("/status", methods=["GET"])
def status():
    return jsonify({"ok": True, "message": "arm server running"})


@app.route("/poses", methods=["GET"])
def list_poses():
    squares = sorted(get_arm().poses.keys())
    return jsonify({"count": len(squares), "squares": squares})


@app.route("/poses/<square>", methods=["GET"])
def get_pose(square):
    square = square.upper()
    poses = get_arm().poses
    if square not in poses:
        return jsonify({"error": f"unknown square: {square}"}), 404
    return jsonify({square: poses[square]})


@app.route("/move", methods=["POST"])
def move():
    data = request.get_json()
    square = data.get("square", "").upper()
    phase = data.get("phase", "hover")
    hand = data.get("hand", 2.78)

    a = get_arm()
    if square not in a.poses:
        return jsonify({"error": f"unknown square: {square}"}), 404
    if phase not in a.poses[square]:
        return jsonify({"error": f"unknown phase: {phase}"}), 400

    pose = a.poses[square][phase]
    a.move_to(pose, hand)
    return jsonify({"ok": True, "square": square, "phase": phase, "pose": pose})


@app.route("/pick", methods=["POST"])
def pick():
    data = request.get_json()
    square = data.get("square", "").upper()

    a = get_arm()
    if square not in a.poses:
        return jsonify({"error": f"unknown square: {square}"}), 404

    a.pick(square)
    return jsonify({"ok": True, "action": "pick", "square": square})


@app.route("/place", methods=["POST"])
def place():
    data = request.get_json()
    square = data.get("square", "").upper()

    a = get_arm()
    if square not in a.poses:
        return jsonify({"error": f"unknown square: {square}"}), 404

    a.place(square)
    return jsonify({"ok": True, "action": "place", "square": square})


@app.route("/pick_and_place", methods=["POST"])
def pick_and_place():
    data = request.get_json()
    from_sq = data.get("from", "").upper()
    to_sq = data.get("to", "").upper()
    a = get_arm()
    if from_sq not in a.poses:
        return jsonify({"error": f"unknown square: {from_sq}"}), 404
    if to_sq not in a.poses:
        return jsonify({"error": f"unknown square: {to_sq}"}), 404

    a.pick_and_place(from_sq, to_sq)
    return jsonify({"ok": True, "from": from_sq, "to": to_sq})


@app.route("/adjust", methods=["POST"])
def adjust():
    data = request.get_json()
    square = data.get("square", "").upper()
    phase = data.get("phase")
    joint = data.get("joint")
    delta = data.get("delta", 0)

    a = get_arm()
    if square not in a.poses:
        return jsonify({"error": f"unknown square: {square}"}), 404
    if phase not in a.poses[square]:
        return jsonify({"error": f"unknown phase: {phase}"}), 400

    pose = a.poses[square][phase]
    if joint not in pose:
        return jsonify({"error": f"unknown joint: {joint}"}), 400

    old_val = pose[joint]
    pose[joint] = round(old_val + delta, 6)

    a.move_to(pose, data.get("hand", 2.78))

    return jsonify(
        {
            "ok": True,
            "square": square,
            "phase": phase,
            "joint": joint,
            "old": old_val,
            "new": pose[joint],
            "delta": delta,
        }
    )


@app.route("/set", methods=["POST"])
def set_joint():
    data = request.get_json()
    square = data.get("square", "").upper()
    phase = data.get("phase")
    joint = data.get("joint")
    value = data.get("value")

    a = get_arm()
    if square not in a.poses:
        return jsonify({"error": f"unknown square: {square}"}), 404
    if phase not in a.poses[square]:
        return jsonify({"error": f"unknown phase: {phase}"}), 400

    pose = a.poses[square][phase]
    if joint not in pose:
        return jsonify({"error": f"unknown joint: {joint}"}), 400

    old_val = pose[joint]
    pose[joint] = value

    a.move_to(pose, data.get("hand", 2.78))

    return jsonify(
        {
            "ok": True,
            "square": square,
            "phase": phase,
            "joint": joint,
            "old": old_val,
            "new": value,
        }
    )


@app.route("/save", methods=["POST"])
def save():
    a = get_arm()
    with open(POSES_PATH, "w") as f:
        json.dump(a.poses, f, indent=2)
    return jsonify({"ok": True, "saved": len(a.poses), "path": POSES_PATH})


@app.route("/home", methods=["POST"])
def home():
    get_arm().move_init()
    return jsonify({"ok": True, "action": "home"})


@app.route("/raw", methods=["POST"])
def raw():
    data = request.get_json()
    resp = get_arm().send(data)
    return jsonify({"ok": True, "sent": data, "response": resp})


if __name__ == "__main__":
    print("Starting arm server on 0.0.0.0:5000")
    print("Connecting to arm...")
    get_arm()
    print("Ready.")
    app.run(host="0.0.0.0", port=5000, debug=False)
