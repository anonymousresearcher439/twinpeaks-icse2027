from pathlib import Path

for path in Path("./missions").rglob("*.json"):
    mission = str(path).rstrip(".json").lstrip("missions/")
    print(mission)
