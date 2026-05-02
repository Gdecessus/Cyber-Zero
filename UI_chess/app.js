// --- State ---
let paused = false;
let aiThinking = false;

// --- DOM ---
const statusEl = document.getElementById("status");
const aiPanel  = document.getElementById("ai-panel");

// --- Board (chessboard.js) ---
const board = Chessboard("board", {
  position: "start",
  draggable: true,
  pieceTheme: "vendor/pieces/{piece}.png",
  onDrop: function(from, to) {
    if (paused) return "snapback";
    if (aiThinking) return "snapback";
    sendMove(from, to);
  }
});

window.addEventListener("resize", function() { board.resize(); });


// --- Backend calls ---
function fetchState() {
  fetch("/state").then(function(r) { return r.json(); }).then(function(data) {
    board.position(data.fen.split(" ")[0], false);
    if (data.game_over) {
      statusEl.textContent = "Game over: " + data.result;
    }
  });
}

function sendMove(from, to) {
  statusEl.textContent = "Sending move...";
  fetch("/move?uci=" + from + to).then(function(r) { return r.json(); }).then(function(data) {

    if (!data.ok) {
      statusEl.textContent = "Illegal move, try again";
      fetchState();
      return;
    }

    fetchState();
    statusEl.textContent = "AI is thinking...";
    aiThinking = true;
    fetch("/ai").then(function() {
      aiThinking = false;
      fetchState();
      if (!paused) statusEl.textContent = "Your turn";
    });
  });
}


// --- Buttons ---
document.getElementById("btnPause").addEventListener("click", function() {
  paused = true;
  statusEl.textContent = "Paused";
});

document.getElementById("btnResume").addEventListener("click", function() {
  paused = false;
  statusEl.textContent = "Your turn";
});

document.getElementById("btnReset").addEventListener("click", function() {
  paused = false;
  fetch("/reset").then(function() {
    fetchState();
    statusEl.textContent = "Your turn";
  });
});


// --- Live AI thinking panel (polled every 500ms) ---
setInterval(updateThinking, 500);

function updateThinking() {
  fetch("/thinking").then(function(r) { return r.text(); }).then(function(text) {
    document.getElementById("ai-panel").innerHTML = text;
  });
}

// --- Game history panel (polled every second) ---
setInterval(updateHistory, 1000);

function updateHistory() {
  fetch("/history").then(function(r) { return r.text(); }).then(function(text) {
    document.getElementById("history-panel").innerHTML = text;
  });
}


// --- Start ---
fetchState();
