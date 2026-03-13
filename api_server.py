from fastapi import FastAPI
import subprocess

app = FastAPI()

@app.post("/replay")
def replay():
    subprocess.run(["python", "replay_poses.py"])
    return {"message": "Replay complete"}
