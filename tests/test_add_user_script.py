"""Tests for the add_user helper script."""

import scripts.add_user as add_user


def test_collect_user_data(monkeypatch):
    inputs = iter(
        ["Alice", "1990-01-01", "123456789", "alice@example.com", "1"]
    )
    monkeypatch.setattr("builtins.input", lambda _: next(inputs))
    data = add_user.collect_user_data()
    assert data == {
        "name": "Alice",
        "birthday": "1990-01-01",
        "phone": "123456789",
        "email": "alice@example.com",
        "level": 1,
    }


def test_capture_photos_without_cv2(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(add_user, "cv2", None)
    add_user.capture_photos(tmp_path, 3)
    assert not list(tmp_path.iterdir())
    assert "OpenCV is not installed" in capsys.readouterr().out


def test_train_user_photos_without_face_recognition(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(add_user, "face_recognition", None)
    add_user.train_user_photos(tmp_path, "Alice")
    assert "face_recognition is not installed" in capsys.readouterr().out
