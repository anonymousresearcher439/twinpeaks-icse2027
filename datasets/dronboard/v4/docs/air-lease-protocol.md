# Drone Air Leasing Protocol

## 1. Overview
### Purpose and Scope
The Air Leasing Protocol prevents drones from crashing into each other. Drones request permission to use a specific volume of air space from a ground station. The ground station either approves or denies these requests based on whether the airspace is already in use.

This protocol:

- Lets drones request exclusive access to a volume of airspace
- Ensures no two drones are assigned overlapping airspace
- Works over unreliable network connections

Each drone must remain within its active air lease to avoid collisions with other drones. A drone can hold only one active air lease at any time. The protocol lets drones:

- Request their first air lease before beginning flight
- Replace their current air lease with a new one as they travel
- Give up their air lease upon landing

The 3D volumes aren't just narrow paths—they're three-dimensional volumes that include both the drone's planned trajectory and a safety buffer around it. This buffer accommodates flight path deviations caused by several factors, with GPS error typically being the most significant. Other factors include environmental conditions like wind, control system variations, and other sensor inaccuracies.

The protocol was originally designed for multi-drone missions with a single operator. However, it can support drones operated by different parties so long as they're connected to the same ground station. The protocol can be used to manage airspace anywhere on Earth. The protocol itself imposes no limits on the number of drones or the size of the air space being manage, though individual implementations may add their own restrictions.

We designed the protocol for (semi)autonomous drones with high-level navigation authority. It works when software decides on where to go next and what path to take. When the navigation plan is modeled digitally, the software can then calculate a precise 3D volume of airspace and request it. We intend for the drones to request only the minimal airspace needed for their next movement.

While human pilots could theoretically use the protocol, it's not  very practical. They would either need to request unnecessarily large volumes or rely on additional software to generate precise airspace requests. We designed it with the expectation that software would frequently request tiny airspaces and these requests require complex calculations. 

### Key Components

The Air Leasing Protocol requires the following components:

#### Drones / Software Pilots
Drones must have software (either onboard or ground-based) that:

- Has precise information about planned flight maneuvers
- Can calculate the exact 3D airspace needed for each maneuver
- Waits for airspace approval before executing a maneuver
- Generates appropriate airspace requests

Practically speaking the protocol expects the drones to be controlled by software rather than human pilots.

#### Ground Station
A central service that responds to air lease requests from multiple drones. The ground station:

- Tracks all currently approved air leases
- Approves new air lease requests when it's safe to do so
- Denies air lease requests when it overlaps with an existing air lease allocation

#### Message Transport
The protocol requires a messaging system that can deliver complete JSON messages between the drones and the ground station. While we currently use MQTT, the protocol is transport-agnostic and could work with any system that can reliably deliver whole messages.

The protocol is designed to work in environments with:

- Unreliable message delivery (messages may be lost)
- No guaranteed message order (messages arrive out of order)
- Variable latency
- duplicate messages
- Potential connection interruptions

The protocol does _not_ rely on message transport layer to guarantee delivery or quality-of-service. Instead, it implements its own mechanisms to handle these network challenges.

### High-level Flow

This is the basic step-by-step process for how the Air Leasing Protocol works when nothing goes wrong. This is known as the happy path. The protocol also includes logic to handle lost messages, out-of-order delivery, and other network faults, but those details are not covered here.

The Air Leasing Protocol follows these steps:

1. Before takeoff, a drone calculates the 3D airspace it needs for its first maneuver and sends a request to the ground station. This 3D volume encompasses both the drone’s starting position and all the space it needs to complete its takeoff maneuver.
2. The ground station checks if the requested airspace can be allocated according to its approval rules. At a minimum, it checks that the airspace does _not_ conflict with existing allocations.
3. If the ground station decides to approve the request, it immediately records the air lease in its memory. The air lease takes effect at this moment, even before the drone is notified.
4. The ground station sends a response message to the drone. This message says whether the request was approved or denied.
5. If approved, the drone uses the newly leased airspace. If denied, it may wait and retry or request a different volume of airspace. 
6. When the drone needs to fly outside its air lease, it requests a replacement. The request specifies a new 3D volume that includes both the drone’s current position and all the space it needs to complete its next maneuver. This new volume must overlap with the existing air lease around the drone's current position. The overlapping volume typically looks like a bubble of airspace centered on the drone’s current position.
7. The ground station checks if the requested airspace can be allocated according to its approval rules. At a minimum, it makes sure the requested airspace does _not_ conflict with any air leases held by other drones. The drone’s own air lease is ignored during this check.
8. If the ground station approves the request, it immediately swaps the existing air lease with the new one by updating its memory. The swap takes effect right away — the old lease ends, and the new lease starts immediately, even before the response message is sent.
9. The ground station sends a response message to the drone. This message says whether the swap was approved or denied.
10. Steps 5 through 9 repeat until the drone receives the air lease needed for its final descent to land. This descent, including touchdown, is covered by an ordinary air lease just like any other maneuver.
11. After the drone touches down, it sends the land message.
12. When the ground station receives the land message, it ends the drone's air lease. The drone no longer holds any airspace after the ground station updates its memory.

Sequence Diagram showing High-level flow

[![](https://mermaid.ink/img/pako:eNqdlE1z2jAQhv_Kjk5kxu0P8CEzTGl7SXNIJpeOL1tpDZrIK0cfUCaT_961MQYChFBfjIX0vvvxaF-V9oZUqSK9ZGJNM4vzgE3FIE-LIVltW-QEs-CZjpd_Bp_ZPCZM1nPFmw393i-3twd_lvDQecQEaENsURNMLGuXDUXQOQQSudZH222-2QgdCBwLfl-iy5gIwqA80QvSz1D7ANpz7axOcZBCl2C6NZ4u0Tr844aEPmH1QNoHIyrO634JJpIGOMJIEJNUJAL71c15wb4oJUzbNvjlGPJuP-pkl10ye5XunnvfJWjniwS-hkHlSVzTggA3amas6e6coVOK5OTkWIcnxsuVGCxnxOvjqE9G179AI8MKbSrkVArrAqQr20YZW9fUd_wwcGKzhch538IvZMpLCvBtrZ1wMpnlYHkOP1xnuVftC8gFap2YNPuOMJHKBYftWfyuQRDZiMt7SGw9tmiv0xdQ_B96PiLoQ4qYVifgOQ_QJyG6GqQLMD0TSZ_or42p6__m3nU1v46yr3tJsBmZgwuT665zaihGnAs2dSBBcbz-AyyHwb8TmI6zIlD_NoWMix2KC3JGFaqh0KA1Mo9fO9FKyR1vqFKl_DRUY3apUhW_yVbMyT-uWasyhUyFEr_5QpU1Sm8KlVsjfRuG-bgqE_u399vvt3-bdgMp?type=png)](https://mermaid.live/edit#pako:eNqdlE1z2jAQhv_Kjk5kxu0P8CEzTGl7SXNIJpeOL1tpDZrIK0cfUCaT_961MQYChFBfjIX0vvvxaF-V9oZUqSK9ZGJNM4vzgE3FIE-LIVltW-QEs-CZjpd_Bp_ZPCZM1nPFmw393i-3twd_lvDQecQEaENsURNMLGuXDUXQOQQSudZH222-2QgdCBwLfl-iy5gIwqA80QvSz1D7ANpz7axOcZBCl2C6NZ4u0Tr844aEPmH1QNoHIyrO634JJpIGOMJIEJNUJAL71c15wb4oJUzbNvjlGPJuP-pkl10ye5XunnvfJWjniwS-hkHlSVzTggA3amas6e6coVOK5OTkWIcnxsuVGCxnxOvjqE9G179AI8MKbSrkVArrAqQr20YZW9fUd_wwcGKzhch538IvZMpLCvBtrZ1wMpnlYHkOP1xnuVftC8gFap2YNPuOMJHKBYftWfyuQRDZiMt7SGw9tmiv0xdQ_B96PiLoQ4qYVifgOQ_QJyG6GqQLMD0TSZ_or42p6__m3nU1v46yr3tJsBmZgwuT665zaihGnAs2dSBBcbz-AyyHwb8TmI6zIlD_NoWMix2KC3JGFaqh0KA1Mo9fO9FKyR1vqFKl_DRUY3apUhW_yVbMyT-uWasyhUyFEr_5QpU1Sm8KlVsjfRuG-bgqE_u399vvt3-bdgMp)

### High-Level Rules

These are the rules that the ground station and drones follow. These rules help us reason about safety.

#### Ground Station Protocol Rules

1. Every request must have a `request_number` that is greater than any `request_number` seen from a given drone.
    - The ground station maintains a table recording the largest `request_number` it has seen for each drone.
    - When a new request arrives, the ground station compares its `request_number` against the stored value:
        - If the new `request_number` is larger, then the request is processed further.
        - If the `request_number` is not larger, the request is denied.
2. Every request must specify the current Lease ID.
    - The drone includes its current Lease ID in every request.
    - If the Lease ID in the request does not match the ground station’s records, the request is denied.
3. Every response from the ground station must include the drone’s current Lease ID.
    - The ground station always includes the drone’s current Lease ID in its response.
4. The ground station sends a response for every request.
    - Open Question: Should the ground station send a response if the `request_number` is out of date?
    - What we do now: The ground station always responds. In case the `request_number` is out of date, then the request is denied.
    - Alternative option: The ground station ignores the message.

#### Drone Protocol Rules

1. Every outgoing request must have a larger `request_number` than any previous one.
    - The drone generates a new `request_number` for each request.
    - The `request_number` must always be larger than any previous request from that drone.
    - The drone never reuses a `request_number`, even if it is sending an otherwise identical request again.
2. The drone keeps a record of every request it sends.
    - Each time the drone sends a request, it saves a record of that request in a table of "pending requests."
    - This record includes the `request_number` and all relevant details.
    - The drone uses this record to track which requests are still waiting for a response.
3. The drone ignores responses to requests it no longer tracks.
    - If a response arrives, the drone first checks if it has a record of the matching `request_number`.
    - If no record exists in the table of pending requests, then the response is ignored.
4. The drone deletes request records only when it is safe to do so.
    - A request record is deleted only if:
        1. The drone receives a direct response to that specific request.
        2. The drone receives a response to a newer request (i.e., a response with a larger `request_number`).
    - These records are deleted from the table of "pending requests."

## Message Types

This section describes all message types in the Air Leasing Protocol, including their purpose, format, and sample messages. The protocol uses JSON for all messages.

### Messages Sent by Drones

#### Air Lease Request Messages
Drones send these messages to request exclusive access to a volume of airspace. There are two types of Air Lease Requests: Single-Tunnel and Multi-Tunnel.

### Single-Tunnel Requests

Sender: Drone

Recipient: Ground Station

topic: `airlease/request`

**Example Message**
```json
{
    "drone_id": "Polkadot",
    "request_number": 1726680466224,
    "current_lease": 1726670400000,
    "start_position": [
        41.606628499839914,
        -86.35600314506233,
        205.5989234039723
    ],
    "end_position": [
        41.60676355052158,
        -86.35609911241892,
        213.5989460930201
    ],
    "radius": 5
}
```

The drone sends this request when it wants a "single-tunnel" air lease. This message specifies a cylindrical-like volume of airspace between two points. Unlike a standard cylinder, The flat end caps are replaced with hemispheres. The volume includes all points that are within `radius` meters of the line connecting `start_position` and `end_position`. This creates a safety buffer around the drone's planned trajectory. The line from `start_position` and `end_position` is the shortest line between these points.

**Fields**

- `drone_id`: Unique identifier for the requesting drone
- `request_number`: Monotonically increasing identifier for each request. Currently we use milliseconds since epoch. A request number may be used exactly once. Even when the drone is sending an otherwise identical request message, this number must change.
- `current_lease`: The "Lease ID" of the drone's current air lease. Must be 0 if the drone doesn't have an active lease
- `start_position`: Starting coordinates [latitude, longitude, altitude] in LLA format with altitude in meters above WGS84 ellipsoid
- `end_position`: Ending coordinates in the same format
- `radius`: Radius of the tunnel in meters, defining the safety buffer around the trajectory

### Multi-Tunnel Air Lease Request
Sender: Drone

Recipient: Ground Station

topic: `airlease/request`

**Example Message**
```json
{
    "requests": [
        {
            "drone_id": "Polkadot",
            "request_number": 1726681252235,
            "current_lease": 1726670400000,
            "start_position": [
                41.606628499839914,
                -86.35600314506233,
                205.5989234039723
            ],
            "end_position": [
                41.60671853356387,
                -86.35612310413713,
                215.59893908943135
            ],
            "radius": 5
        },
        {
            "drone_id": "Polkadot",
            "request_number": 1726681252236,
            "current_lease": 1726670400000,
            "start_position": [
                41.60671853356387,
                -86.35612310413713,
                215.59893908943135
            ],
            "end_position": [
                41.60680856714493,
                -86.35624306319089,
                225.5989547729945
            ],
            "radius": 5

        }
    ]
}
```
A "multi-tunnel" request defines a list of Single-Tunnel Request objects. 

**Fields**

- `requests`: a list of single tunnel request objects.

Note: see the Single-Tunnel Request message for more detail.

Note: The overall `request_number` is taken from the first Single-Tunnel request number in the list. In the example above, the Ground Station would reference `1726681252235` as the `request_number` in it's response. In case a multi-tunnel request is approved, this same `request_number` gets promoted to a Lease ID.

### Air Lease Land Message
Sender: Drone

Recipient: Ground Station

topic: `airlease/land`

```json
{
    "drone_id": "Polkadot",
    "request_number": 1726682252234,
    "current_lease": 1726670400000
}
```
The drone sends the Land message when it's done flying. The air leasing service will remove all of the routes associated with the drone id. 

TODO
- Air Lease Hover Message
- Air Lease Cleanup Message
- Air Lease Query Message


### Air Lease Response message

Sender: Ground Station

Recipient: Drone

topic: `drone/Polkadot/airlease/status`

**Example**
```json
{
    "drone_id": "Polkadot",
    "request_number": 1726681252235,
    "current_lease": 1726681252235,
    "approved": true,
    "deadlock": false
}
```

The air leasing service sends this message to the drone in response to an air lease request. The message contains the drone's `drone_id`, the `request_number`, and a boolean indicating if the request was approved. The `deadlock` property is `true` in case the request is denied and this drone is deadlocked with at least one other drone. Every request will result in a response message.

The `deadlock` property indicates if a drone’s request for an air tunnel has led to a deadlock, where drones block each other in a cyclic dependency. It is set to ‘true’ when the airspace a drone requests is already occupied by another drone, who is similarly obstructed by the first (either directly or indirectly). This results in a cycle where no involved drones can advance.

For instance, suppose Drone A is at position A and Drone B is at position B. If Drone A requests a lease to position B but is denied because Drone B occupies it, then there is no deadlock as Drone B isn't blocked. However, if Drone B then requests an air lease to position A, then both drones obstruct each other. In this case, we have deadlock.

The `current_lease` always specifies the drone's current Lease ID. This is the Lease ID that the drone should use in it's next Air Lease request.

**Fields**

- `drone_id`: Unique identifier for the drone that made the request
- `request_number`: The request number from the original request message. The ground station echoes the `request_number` from the request. If the ground station is responding to a Multi-Tunnel request, then this matches the first request_number in the list.
- **`current_lease`**: The **Lease ID** of the drone's current air lease. This ID identifies the drone's active air lease. The drone must specify this **Lease ID** in its next Air Lease request. In case an air lease was approved, this value will match the `request_number`. (The `request_number` is promoted to a **Lease ID** when the requested lease is approved and the lease starts). This value will be `0` in case the drone does not have an active air lease. This field plays a critical role when recovering from message loss or network interruptions. By including the current lease ID in every response, the protocol ensures drones can resynchronize their state with the ground station even if previous messages were lost.
- `approved`: Boolean indicating whether the air lease request was approved
- `deadlock`: Boolean indicating whether the drone is involved in a deadlock situation

### Query Response: List of Active and Denied Air Leases
TODO 



##  Protocol Flow
- Normal operation sequence
- Message Loss
- Duplicate Messages
- Out of order messages

### Message Loss
How the protocol handles message loss.

Scenario: The drone sends a request, and the request is loss.

Outcome: The drone waits and sends a new request with the same airspace volume

Scenario: The drone sends a request. It's approved and the response is lost

Outcome: The drone sends a new request for the same airspace volume. This new request references the wrong `current_lease`. The Ground station Denies the request. The response message informs the drone that the previous request was approved. The drone registeres that it's current lease is for it's desired airspace volume. The drone uses the airspace volume.


Scenario: The drone sends a request. It's denied and the response is lost

Outcome: The drone sends a new request for the same airspace volume. The Ground station processes it like any other.

### Duplicate messages
The ground station must deny any request with a duplicate request number. Or any request with a request number smaller than the largest request number it's ever seen from that drone. This helps in case duplicate messages arrive.

Scenario 1: the ground station receives the same request twice. The first time it processes the request as usual. The second time it denies the request, but the response will inform the drone of it's current air lease.

The drone must ignore any response that doesn't reference an existing pending request.

Scenario 2: the drone receives the same response message twice. Upon receiving the message for the first time, the drone updates it's state. If the message is a response for the most recent request, then the drone cleans up the pending request. It also cleans up all the other pending requests because they have a `request_number` that's smaller. If the air lease was approved, the drone updates it's current lease. If the air lease was denied, it doesn't update the current lease. The second time this response message arrives, the drone will ignore it because there doesn't exist a record of the pending request.



