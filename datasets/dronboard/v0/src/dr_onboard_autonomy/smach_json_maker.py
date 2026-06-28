import json

smach_machine = {
    "DisarmedOnGround": {
        "outcomes": {
            "armed_event": "ArmedOnGround",
            "error_event": "failure_to_arm",
            "mqtt_smach": "new_state_mach",
        },
        "init": True,
    }
}

print(json.dumps(smach_machine))
