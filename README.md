# Arduino Motor Control

A differential drive controller with visual encoders that receives commands via serial interface.

Can be controlled from a host device application such as [this ROS controller](https://github.com/phildue/bot_base).

The PID control is encapsulated in [this Arduino library](https://github.com/phildue/bot_low_level/tree/master).

## Protocol

```
<type> <verb> [<verb>] [<timestamp>] [<value>]
```

| **Message Type** | **Purpose**                    | **Verbs** | **Purpose**                    | **Verbs** | **Purpose**                        | **Syntax**                                          |
|------------------|--------------------------------|-----------|--------------------------------|-----------|------------------------------------|-----------------------------------------------------|
| set              | Write values to target         | vel       | assign set point velocity      |           |                                    | set vel <timestamp> <vl> <vr>                       |
|                  |                                | dty       | assign set point duty cycle    |           |                                    | set dty <timestamp> <dl> <dr>                       |
|                  |                                | cfg       | assign configuration parameter | kp(l\|r)  | control constant p (left \| right) | set cfg kp <value>                                  |
|                  |                                |           |                                | ki(l\|r)  | control constant i (left \| right) | set cfg ki <value>                                  |
|                  |                                |           |                                | kd(l\|r)  | control constant d (left \| right) | set cfg kd <value>                                  |
|                  |                                |           |                                | t         | command execution time in seconds  | set cfg t <value>                                   |
| query            | Query values from target       | vap       | read position and velocity     |           |                                    | query vap                                           |
|                  |                                | pos       | read position                  |           |                                    | query pos                                           |
|                  |                                | pid       | read pid parameters            |           |                                    | query cfg                                           |
| state            | Send values from target        | vap       | Reply to query vap             |           |                                    | state vap <timestamp> <pl> <vl> <pr> <vr>           |
|                  |                                | pos       | Reply to query pos             |           |                                    | state pos <timestamp> <pl> <pr>                     |
|                  |                                | pid       | Reply to query pid             |           |                                    | state pid <timestamp> <pl> <il> <dl> <pr> <ir> <dr> |
| info             | General human readable message |           |                                |           |                                    | info <timestamp> <message>                          |
