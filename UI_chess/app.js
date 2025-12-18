const statusText = document.getElementById("statusText");
const statusChips = document.getElementById("statusChips");
const turnText = document.getElementById("turnText");
const modeText = document.getElementById("modeText");
const connText = document.getElementById("connText");
const moveLog = document.getElementById("moveLog");
const boardEl = document.getElementById("board");

const btnStart = document.getElementById("btnStart");
const btnSim = document.getElementById("btnSim");
const btnSettings = document.getElementById("btnSettings");
const btnPause = document.getElementById("btnPause");
const btnResume = document.getElementById("btnResume");
const btnReset = document.getElementById("btnReset");

const state = {
  mode: "Not started",
  turn: "-",
  connection: "Offline",
  tags: ["Idle"],
  log: [],
  activeSquares: [],
};

// Unicode chess pieces for the start position
const demoPieces = {
  a1: "♖", b1: "♘", c1: "♗", d1: "♕", e1: "♔", f1: "♗", g1: "♘", h1: "♖",
  a2: "♙", b2: "♙", c2: "♙", d2: "♙", e2: "♙", f2: "♙", g2: "♙", h2: "♙",
  a7: "♟", b7: "♟", c7: "♟", d7: "♟", e7: "♟", f7: "♟", g7: "♟", h7: "♟",
  a8: "♜", b8: "♞", c8: "♝", d8: "♛", e8: "♚", f8: "♝", g8: "♞", h8: "♜",
};

const squares = [];
const files = ["a", "b", "c", "d", "e", "f", "g", "h"];
const ranks = ["8", "7", "6", "5", "4", "3", "2", "1"];

function buildBoard() {
  boardEl.innerHTML = "";
  squares.length = 0;
  ranks.forEach((r, rIdx) => {
    files.forEach((f, fIdx) => {
      const id = f + r;
      const sq = document.createElement("div");
      sq.className = "square " + ((rIdx + fIdx) % 2 === 0 ? "light" : "dark");
      sq.dataset.square = id;
      sq.textContent = demoPieces[id] || "";
      sq.addEventListener("click", () => handleSquareClick(id));
      boardEl.appendChild(sq);
      squares.push(sq);
    });
  });
}

function setStatus(text) {
  statusText.textContent = text;
}

function setTags(tags) {
  statusChips.innerHTML = "";
  tags.forEach((t) => {
    const li = document.createElement("li");
    li.className = "tag";
    li.textContent = t;
    statusChips.appendChild(li);
  });
}

function appendLog(entry) {
  state.log.push(entry);
  moveLog.textContent = state.log.join("   ");
}

function setActiveSquares(list) {
  squares.forEach((sq) => sq.classList.remove("active"));
  list.forEach((id) => {
    const sq = squares.find((s) => s.dataset.square === id);
    if (sq) sq.classList.add("active");
  });
}

function handleSquareClick(id) {
  if (state.activeSquares.length === 0) {
    state.activeSquares = [id];
  } else if (state.activeSquares.length === 1) {
    const from = state.activeSquares[0];
    const to = id;
    state.activeSquares = [];
    handlePlayerMove(from, to);
  }
  setActiveSquares(state.activeSquares);
}

function handlePlayerMove(from, to) {
  appendLog(`Player: ${from}-${to}`);
  setStatus("Move submitted (waiting on backend)");
  state.turn = "Waiting";
  state.tags = ["Pending backend"];
  setTags(state.tags);
  setActiveSquares([from, to]);
  updateTop();
}

function updateTop() {
  turnText.textContent = state.turn;
  modeText.textContent = state.mode;
  connText.textContent = state.connection;
}

btnStart.addEventListener("click", () => {
  state.mode = "Live";
  state.turn = "Player";
  state.connection = "Local";
  state.tags = ["Ready"];
  state.log = [];
  moveLog.textContent = "No moves yet.";
  setStatus("Your turn");
  setTags(state.tags);
  setActiveSquares([]);
  updateTop();
});

btnSim.addEventListener("click", () => {
  state.mode = "Simulation";
  state.connection = "Local";
  setStatus("Simulation ready");
  setTags(["Sim", "Ready"]);
  updateTop();
});

btnSettings.addEventListener("click", () => {
  setStatus("Settings: coming soon");
});

btnPause.addEventListener("click", () => {
  setStatus("Paused");
  state.tags = ["Paused"];
  setTags(state.tags);
});

btnResume.addEventListener("click", () => {
  setStatus("Resumed");
  state.tags = ["Ready"];
  setTags(state.tags);
});

btnReset.addEventListener("click", () => {
  setStatus("Reset");
  state.tags = ["Idle"];
  state.turn = "-";
  state.mode = "Not started";
  state.connection = "Offline";
  state.log = [];
  moveLog.textContent = "No moves yet.";
  setTags(state.tags);
  setActiveSquares([]);
  updateTop();
  buildBoard();
});

buildBoard();
updateTop();
