from flask import Flask, request,g,jsonify
import sqlite3
import datetime 
from api_keys_config import require_appkey
app = Flask(__name__)

DATABASE = 'database.db'

def get_db():
    db = getattr(g, '_database', None)
    if db is None:
        db = g._database = sqlite3.connect(DATABASE)
    return db


def init_db():
    with app.app_context():
        db = get_db()
        with app.open_resource('schema.sql', mode='r') as f:
            db.cursor().executescript(f.read())
        db.commit()


@app.teardown_appcontext
def close_connection(exception):
    db = getattr(g, '_database', None)
    if db is not None:
        db.close()

@app.route("/")
def test():
    return "contaxt server up and running woohoo"


@app.route("/postForceData",methods=["POST"])
@require_appkey
def post_force_data():
    if request.method == "POST":
        db = get_db()
        cur = db.cursor()
        force_data = request.json["force_data"]
        sql = ''' INSERT INTO sensorData (forceReading, dateTimeCreated) VALUES(?,?) '''
        insert_vals = (force_data,datetime.datetime.now())
        cur.execute(sql,insert_vals)
        db.commit()
        return jsonify(success = True)

if __name__ == "__main__":
    init_db()
