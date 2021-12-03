from flask import Flask, render_template
from flask import request
from flask_cors import CORS, cross_origin
import astar

app = Flask(__name__)
CORS(app)

@app.route('/')
def index():
    return render_template("index.html")

@app.route('/pathfind', methods=['GET', 'POST'])
@cross_origin(supports_credentials=True)
def pathfind():

    dt = request.get_json( request.args.get('data') )

    return astar.run(dt)



if __name__ == "__main__":
    app.run(debug=True)