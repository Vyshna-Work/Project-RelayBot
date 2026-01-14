from flask import Flask, request, jsonify, render_template, g
import sqlite3
import requests

app = Flask(__name__)

DATABASE = "relaybot.db"

# ---------------------------------------------------
# RENDER ENDPOINT 
# ---------------------------------------------------
RENDER_URL = "https://git-test-pprd.onrender.com/relaybot-data"

# ---------------------------------------------------
# GET A NEW DB CONNECTION FOR EACH REQUEST
# ---------------------------------------------------
def get_db():
    if "db" not in g:
        g.db = sqlite3.connect(DATABASE)
        g.db.row_factory = sqlite3.Row
    return g.db

@app.teardown_appcontext
def close_db(exception):
    db = g.pop("db", None)
    if db is not None:
        db.close()

# ---------------------------------------------------
# INITIALIZE DATABASE ON STARTUP
# ---------------------------------------------------
def init_db():
    db = sqlite3.connect(DATABASE)
    cur = db.cursor()

    cur.execute("""
    CREATE TABLE IF NOT EXISTS telemetry (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        botID TEXT,
        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
        left_speed INTEGER,
        right_speed INTEGER,
        state TEXT
    )
    """)

    cur.execute("DELETE FROM telemetry")
    db.commit()
    db.close()

    print("Database initialized and cleared.")

init_db()

# ---------------------------------------------------
# RECEIVE TELEMETRY FROM ARDUINO → STORE LOCALLY → FORWARD TO RENDER
# ---------------------------------------------------
@app.route("/relaybot-data", methods=["POST"])
def relaybot_data():
    print("RAW:", request.data)

    data = request.get_json(silent=True)
    print("PARSED:", data)

    if data is None:
        return jsonify({"status": "invalid json"}), 400

    # ---- Store locally ----
    db = get_db()
    cur = db.cursor()

    cur.execute("DELETE FROM telemetry")
    cur.execute("""
        INSERT INTO telemetry (botID, left_speed, right_speed, state)
        VALUES (?, ?, ?, ?)
    """, (data["bot"], data["left"], data["right"], data["state"]))
    db.commit()

    print("Stored locally:", data)

    # ---- Forward to Render ----
    try:
        r = requests.post(RENDER_URL, json=data, timeout=1)
        print("Forwarded to Render:", r.status_code)
    except Exception as e:
        print("Render unreachable:", e)

    return jsonify({"status": "ok"})

# ---------------------------------------------------
# RETURN LATEST TELEMETRY (LOCAL)
# ---------------------------------------------------
@app.route("/latest")
def latest():
    db = get_db()
    cur = db.cursor()

    cur.execute("""
        SELECT botID, left_speed, right_speed, state, timestamp
        FROM telemetry
        ORDER BY id DESC
        LIMIT 1
    """)

    row = cur.fetchone()

    if row is None:
        return jsonify({"status": "no data yet"})

    return jsonify({
        "bot": row["botID"],
        "left": row["left_speed"],
        "right": row["right_speed"],
        "state": row["state"],
        "timestamp": row["timestamp"]
    })

# ---------------------------------------------------
# ROUTES
# ---------------------------------------------------
@app.route("/dashboard")
def dashboard():
    return render_template("dashboard.html")

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/team")
def team():
    return render_template("team.html")

@app.route("/network")
def network():
    return render_template("network.html")

# ---------------------------------------------------
# RUN APP
# ---------------------------------------------------
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
