# How to run the integration tests
To run `launch/test_message_senders.launch` use:

```bash
rostest dr_onboard_autonomy test_message_senders.launch
```

To run `launch/test_preflight.launch` some extra setup is needed. make sure PX4 is in your catkin src:

https://docs.px4.io/master/en/test_and_ci/integration_testing.html
