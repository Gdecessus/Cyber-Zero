"""
Test squares by picking a piece and placing it on the next one.

Usage:
    python -m src.robot.test_board           # start from A1
    python -m src.robot.test_board D1        # start from D1
    python -m src.robot.test_board D1 F1     # just D1 to F1
"""

import sys
from src.robot.arm import Arm

FILES = "ABCDEFGH"
arm = Arm()

# build full square list
all_squares = []
for rank in range(1, 9):
    for file_char in FILES:
        all_squares.append(file_char + str(rank))

args = sys.argv[1:]

if len(args) == 2:
    # specific range: from_sq to_sq
    start = args[0].upper()
    end = args[1].upper()
    if start in all_squares and end in all_squares:
        i = all_squares.index(start)
        j = all_squares.index(end)
        squares = all_squares[i:j+1]
    else:
        print(f"Unknown square: {start} or {end}")
        sys.exit(1)
elif len(args) == 1:
    # start from this square
    start = args[0].upper()
    if start in all_squares:
        squares = all_squares[all_squares.index(start):]
    else:
        print(f"Unknown square: {start}")
        sys.exit(1)
else:
    squares = all_squares

print(f"Testing {len(squares)} squares: {squares[0]} to {squares[-1]}")
print(f"Place a piece on {squares[0]} and press ENTER to start...")
input()

for i in range(len(squares) - 1):
    from_sq = squares[i]
    to_sq = squares[i + 1]
    print(f"\n{from_sq} -> {to_sq}  ({i+1}/{len(squares)-1})")
    arm.pick_and_place(from_sq, to_sq)

    result = input("  ENTER=next  q=quit: ").strip().lower()
    if result == "q":
        break

print("\nDone.")
