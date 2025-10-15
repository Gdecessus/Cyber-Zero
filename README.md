# Chess Neural Network + Robot Arm

Final year project - Building a chess AI that can actually move pieces with a robot arm.

## What I'm Building

Basically combining AlphaZero-style chess AI with a 6DOF robot arm. The AI learns to play chess (through both learning from real games and playing against itself), then uses ROS2 to control the robot arm to physically move the pieces.

## The Goal

- Get the AI to play decent chess (aiming for around 1500 ELO) through reinforced and supervised learning
- Hook it up to the arm via ROS2
- Make it actually work in real life (physical board + pieces)
- Have the final prototype able to play a game against a real person through a GUI (Raspberry Pi)

## Tech Stack

**AI Stuff:**
- TensorFlow for the neural network
- MCTS (Monte Carlo Tree Search) for move selection
- Python-chess library for game rules and such
- Arena evaluation to decide winner model (best model stays)

**Robot Stuff:**
- ROS2 Humble 
- 6DOF arm (already have this)
- Raspberry Pi + Raspberry Pi Touch Screen
- Physical chessboard 30x30cm with a  10cm support for the arm and 2cm gap between support and board

**Other:**
- Python 3.9
- Git (obviously)
- Unbuntu (Ros2 runs on Unbuntu)
- Lots of white monster

## Project Structure
```
src/
├── ai_engine/      # Neural network, MCTS, training code
├── ros2_stuff/     # ROS2 nodes and messages
└── utils/          # Helper functions

docs/               # Documentation (will fill this in as I go)
tests/              # Tests (TODO: actually write tests near the end for marks)
```

## Timeline (rough plan)

- Nov-Dec: Build the AI engine, get it playing chess
- Jan-Feb: ROS2 integration, connect AI to robot through ROS2 Nodes
- Mar-Apr: Get the robot actually picking up pieces without dropping them
- May: Final testing, documentation, pray it works for demo

## Current Status

I've done about 7 months of research through my internship, I've set my mind on what's needed, I've played around with this very same 6DOF robotic arm in different pick and place tasks. I've created games in Unity both 2D and 3D so this is the perfect project to put all of my hobbies and passion into one single final year project

## Setup

Will add proper setup instructions once I have something working. For now it's just:
```bash
pip install -r requirements.txt
```

---

Last updated: Oct 15 2024