"""Ask a running RLBotServer to start the validation match.

    python rlbot/start_match.py [match.toml]

The server reads the config file itself, so the path must be readable by the
account that runs RLBotServer.
"""

import sys
from pathlib import Path

from rlbot.managers import MatchManager

if __name__ == "__main__":
    config = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent / "match.toml"
    manager = MatchManager()
    manager.start_match(config.resolve(), wait_for_start=True, ensure_server_started=False)
    print("match started")
    manager.disconnect()
