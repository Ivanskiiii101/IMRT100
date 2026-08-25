import subprocess
from pathlib import Path

AUDIO_FILE = Path(__file__).with_name("meow-meow-meow-tiktok.mp3")

subprocess.Popen(["afplay", str(AUDIO_FILE)])
