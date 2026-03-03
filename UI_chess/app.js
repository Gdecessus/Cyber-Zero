// --- DOM elements ---
const boardEl = document.getElementById("board");
const statusEl = document.getElementById("status");

// --- State ---
let selected = null;   // the square the player clicked first (e.g. "e2")
let paused = false;    // when true, clicks are ignored

// unicode chess pieces keyed by FEN character
const PIECES = {
  K: "\u2654", Q: "\u2655", R: "\u2656", B: "\u2657", N: "\u2658", P: "\u2659",
  k: "\u265A", q: "\u265B", r: "\u265C", b: "\u265D", n: "\u265E", p: "\u265F",
};


// --- Render ---

function renderBoard(fen) {
  // takes a FEN string like "rnbqkbnr/pppppppp/..." and draws the 8x8 grid
  boardEl.innerHTML = "";
  const rows = fen.split(" ")[0].split("/");

  for (let r = 0; r < 8; r++) {
    let col = 0;
    for (const ch of rows[r]) {
      if (ch >= "1" && ch <= "8") {
        // digit means that many empty squares
        for (let i = 0; i < parseInt(ch); i++) {
          addSquare(r, col, null);
          col++;
        }
      } else {
        // letter means a piece
        addSquare(r, col, ch);
        col++;
      }
    }
  }
}

function addSquare(row, col, piece) {
  // create one square on the board, attach a click handler
  const files = "abcdefgh";
  const name = files[col] + (8 - row);          // e.g. "e2"
  const isLight = (row + col) % 2 === 0;

  const sq = document.createElement("div");
  sq.className = "square " + (isLight ? "light" : "dark");
  sq.dataset.square = name;
  sq.textContent = piece ? PIECES[piece] : "";
  sq.addEventListener("click", () => handleClick(name));
  boardEl.appendChild(sq);
}

function setStatus(text) {
  // update the status line above the board
  statusEl.textContent = text;
}

function clearSelection() {
  // remove the green highlight from all squares
  selected = null;
  document.querySelectorAll(".square.selected").forEach(s => s.classList.remove("selected"));
}

function highlightSquare(name) {
  // add the green highlight to one square
  const sq = document.querySelector(`[data-square="${name}"]`);
  if (sq) sq.classList.add("selected");
}


// --- Backend calls ---

async function fetchState() {
  // ask the server for the current board and redraw it
  const res = await fetch("/state");
  const data = await res.json();
  renderBoard(data.fen);

  if (data.game_over) {
    setStatus("Game over: " + data.result);
  }
  return data;
}

async function sendMove(from, to) {
  // send the player's move to the server, then ask the AI to respond
  setStatus("Sending move...");
  const uci = from + to;
  const res = await fetch("/move?uci=" + uci);
  const data = await res.json();

  if (!data.ok) {
    setStatus("Illegal move, try again");
    return;
  }

  // refresh the board after the player's move
  const state = await fetchState();
  if (state.game_over) return;

  // now let the AI play
  setStatus("AI is thinking...");
  await fetch("/ai");
  await fetchState();

  if (!paused) setStatus("Your turn");
}


// --- Click handling ---

function handleClick(name) {
  // two-click system: first click selects a piece, second click picks the destination
  if (paused) return;

  if (!selected) {
    // first click: select the piece
    selected = name;
    highlightSquare(name);
  } else {
    // second click: try to move from selected -> name
    const from = selected;
    clearSelection();
    sendMove(from, name);
  }
}


// --- Buttons ---

document.getElementById("btnPause").addEventListener("click", () => {
  // pause the game so clicks are ignored
  paused = true;
  clearSelection();
  setStatus("Paused");
});

document.getElementById("btnResume").addEventListener("click", () => {
  // resume the game so clicks work again
  paused = false;
  setStatus("Your turn");
});

document.getElementById("btnReset").addEventListener("click", async () => {
  // start a fresh game
  paused = false;
  clearSelection();
  await fetch("/reset");
  await fetchState();
  setStatus("Your turn");
});


// --- Start ---
fetchState();
