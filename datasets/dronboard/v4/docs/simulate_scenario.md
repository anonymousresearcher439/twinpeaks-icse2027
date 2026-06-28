# How to Simulate a Scenario
You simulate a scenario where each drone's starting location is specified in a CSV file.

## Branch
Currently this functionality is only in `dev-502-sade` branch of DR-Onboard.

## Setup

Create a python virtual environment with the dependencies
```bash
cd DR-OnboardAutonomy
python3 -m venv venv
source venv/bin/activate 
pip install -r requirements-simulation.txt
```

## How to Make A Scenario

To create a scenario, first make a CSV file that lists every drone and their
starting location.

[Here's a Google Sheets Template](https://docs.google.com/spreadsheets/d/1gkE2x_ZgDRJ7Vq-eEeeJkSMLtEpgP4FQjnlEYkxe_RY/edit?gid=0#gid=0)

**For Example:**
```csv
name,latitude,longitude
Red,41.86249157864725,-85.90916072039147
Lime,41.862487597450475,-85.90881435882085
Aqua,41.86315069778483,-85.90859499569014
Fuchsia,41.862439891031535,-85.90762196704833
DodgerBlue,41.8616817143815,-85.90767587612963
Gold,41.86142878175984,-85.90920907194518
Orange,41.86404756251015,-85.90928711313124
Violet,41.863084366723044,-85.90952740038739
```

NOTE: the heading (`name,latitude,longitude`) is required.

You will need to save your CSV file somewhere.


## How to Run:

Use the `just` recipe called `simulate-scenario` to run the simulation/

For example,
- if your `.csv` file is at `./scenario-01.csv`
- if you want to elevation to be 274.8

Then you'd run:
```bash
just simulate-scenario scenario-01.csv 274.8
```

NOTE: you might need to activate your virtual environment in order for this to work:
```bash
source venv/bin/activate
just simulate-scenario scenario-01.csv 274.8
```
