# Project Charter

## Basic Info

**Project:** Cyber-Zero  
**Student:** Gustavo Carneiro Da Costa
**Student Number:** x22104020 
**Supervisor:** NA  
**Timeline:** Sept 2024 - May 2025

## What's This About?

Building a chess-playing robot. The AI part learns chess using neural networks and self-play (inspired by AlphaZero and Leela Zero), then the robot arm physically moves the pieces on a real board. During the showcase in May players will be allowed to play against the AI, The Robotic arm will perform both movements, players will use the GUI on a Raspberry Pi Screen.

## Why Am I Doing This?

I was always fascinated about robotics, AI and video games, so during my internship I brain stormed ideas of what I could do that put all of those together and came with this idea after reading the papers on Alpha-Zero and after diving deep into the open source Leela-Zero. Although the complexity of those two chess engines wont be replicated, not even the programming language will, I plan to use the concepts used by both in a python version.


## Main Objectives

**The AI Side:**
- Build a neural network that can evaluate chess positions
- Implement MCTS for searching through possible moves (I've seen a example of MCTS being used to clear the Mario game)
- Train it using real chess games (supervised learning)
- Let it play against itself to get better (reinforcement learning)
- Try to reach ~1500 ELO (Hoping it can beat anyone on showcase day xD)

**The Robot Side:**
- Get ROS2 working with the AI(through nodes)
- Make the arm pick up and place pieces accurately(Do gazeebo simulations first)
- 3D print the pieces I drew on Blender for easier grip
- Handle the basic chess moves (regular moves, captures, maybe castling if there's time)

**Overall:**
- Make it all work together
- Actually demonstrate it working
- Write decent documentation
- Get my diploma

## What's Realistic vs What Would Be Nice

**Definitely doing:**
- AI engine that plays chess
- ROS2 integration
- Robot moves pieces
- Full documentation

**Probably not doing:**
- Computer vision for detecting moves (I thought about this, but I'll go with a full deterministic approach instead)
- Super high ELO rating (>2000)
- Fancy GUI (I'll use built in python libraries, I cant be reiventing the wheel)
- Beautifuly engineered chessboard and support for the arm(I'm not a carpenteer)

## How I'll Know It's Done

- AI plays chess at ~1500 ELO or better
- Robot successfully moves pieces without breaking things
- Can play a full game from start to finish
- Everything is documented
- Demo works for final presentation

## Risks

**Things that might go wrong:**
- AI training takes forever (that's why I'm doing supervised training at start as I dont have compute power to do reinforced learning on its own)
- Robot arm doesnt behave like I want outside the ROS2 simulation.
- Running out of time (Decrese complexity and go for more robotics than AI)
- ROS2 being annoying (Unbuntu CLI can be annoying to learn)

## Schedule (Rough)

**Phase 1 (Sept-Dec):** Get AI working
- Implement neural network
- Get MCTS working
- Train on some games
- Make sure it can play decent chess

**Phase 2 (Jan-Feb):** ROS2 Integration
- Set up ROS2 Simulation
- Connect AI to robot through nodes
- Basic communication working

**Phase 3 (Mar-Apr):** Physical Testing
- Get robot moving pieces
- Debug all the issues that will definitely come up
- Lots of unit testing for marks

**Phase 4 (May):** Finish Up
- Final testing
- Write documentation
- Prepare demo

## Resources

**What I Have:**
- Laptop (good enough for TensorFlow)
- WaveShare 6DOF arm
- Chess set
- Time (hopefully)

**What I Need:**
- Raspberry Pi + Screen
- Structure to support the robot
- 3D print the pieces and 30x30cm board
- Rent some GPU on cloud to train AI if needed

---

