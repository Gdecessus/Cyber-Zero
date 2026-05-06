import pytest
from src.server.server import app


@pytest.fixture
def client():
    app.config["TESTING"] = True
    with app.test_client() as c:
        yield c


def test_state_returns_json(client):
    client.get("/reset")
    resp = client.get("/state")
    assert resp.status_code == 200
    data = resp.get_json()
    assert data["turn"] == "w"
    assert data["game_over"] is False


def test_valid_move_accepted(client):
    client.get("/reset")
    resp = client.get("/move?uci=e2e4")
    assert resp.get_json()["ok"] is True


def test_illegal_move_rejected(client):
    client.get("/reset")
    resp = client.get("/move?uci=e2e5")
    assert resp.get_json()["ok"] is False


def test_invalid_format_rejected(client):
    client.get("/reset")
    resp = client.get("/move?uci=xyz")
    assert resp.get_json()["ok"] is False


def test_reset_clears_board(client):
    client.get("/move?uci=e2e4")
    client.get("/reset")
    data = client.get("/state").get_json()
    assert len(data["legal_moves"]) == 20
