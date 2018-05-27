# ROS gateway

A minimalistic GDP gateway for ROS. In a nutshell, these gateways should enable
one to connect multiple ROS networks together securely in somewhat real time AND
maintain a record of the topics for later analysis.

- `gdp_sink.py`: A ROS node that listens to specified topics and logs them to a
  specified GDP log.
- `gdp_source.py`: A ROS node that subscribes to a given GDP log and publishes
  the contents to appropriate ROS topics.

## Quick overview

`gdp_sink` is essentially a ROS node initiated with a list of topics and a GDP
log name as the arguments. It subscribes to the specified topics, serializes
them and writes them to the GDP log.  From the perspective of a ROS environment,
`gdp_sink` effectively acts as an information sink.

`gdp_source` is another ROS node that can be started either in the same ROS
environment, or in a different ROS environment. Provided a GDP log name, it
subscribes to the log and publishes the incoming messages on topic names
constructed from the original topic names.


```
       -----------------                       -----------------------
      |  ROS master 1   |                     |   ROS master 2        |
      | topics:         |                     |              topics:  |
      | /t1             |        -----        |              /gdp/t1  |
      | /t2     gdp_sink| ====> | GDP |=====> |gdp_source    /gdp/t2  |
       -----------------         -----         -----------------------
```


## Software requirements/installation

At the very least, you will need GDP client side libraries (C library and Python
bindings). You will also need ROS-related setup for making `rospy` work.

While the GDP is fairly reasonably documented in the [GDP
wiki](https://gdp.cs.berkeley.edu/redmine/projects/gdp/wiki), here is a
high-level summary. Please see GDP documentation for in-depth details.

- First, acquire the GDP software from the
  [repository](https://gdp.cs.berkeley.edu/redmine/projects/gdp/wiki/Repo_access).
  The GDP code is under somewhat incompatible changes at the moment, so we use a
  relatively old commit git commit `6c82c4a`.

- Next, install the necessary dependencies using `adm/gdp-setup.sh`.

- Use `sudo make install-python`. This installs the necessary C-libraries and
  the Python wrappers in the system path.

- Next, we need to configure which GDP installation should we use. Use the
  following if you want to use the infrastructure setup in Berkeley.

    ```
    sudo mkdir /etc/ep_adm_params
    sudo sh -c 'echo "swarm.gdp.zeroconf.enable=false" > /etc/ep_adm_params/gdp'
    sudo sh -c 'echo "swarm.gdp.routers=gdp-03.eecs.berkeley.edu; gdp-02.eecs.berkeley.edu; gdp-01.eecs.berkeley.edu; gdp-04.eecs.berkeley.edu" >> /etc/ep_adm_params/gdp'
    ```

## Usage

Before starting either `gdp_sink` or `gdp_source`, you need to create a log.
Although you can create logs with any name at the moment, please stick to the
convention of using a prefix like `edu.berkeley.foggdp.`, so we can avoid name
collisions. Use the following to create a log:

```
gcl-create -e none edu.berkeley.foggdp.[some-identifier]
```

This should create a private signature key (`.pem` file) in your current
directory. Keep this file secure; this is needed to write anything to the log
you just created. This file should be placed in the directory you intend to
execute `gdp_sink.py` from.


Next, assuming you have a ROS master running, you can simply execute something
like following to start logging topics `/t1` and `/t2`:

```
python gdp_sink.py -t /t1 -t /t2 edu.berkeley.foggdp.[some-identifier]
```

Elsewhere, with a different ROS master, you can run `gdp_source.py` as follows,
which should publish messages originally received on `/t1` to `/gdp/t1`, and on
`/t2` to `/gdp/t2`.

```
python gdp_source.py edu.berkeley.foggdp.[some-identifier]
```

For more options and command line flags, use `python gdp_sink.py -h` and `python
gdp_source.py -h`.


## Final notes

- The current GDP implementation provides very minimal security and durability
  guarantees. This will be changed sometime soon. As such, please do not use it
  for sensitive/critical data right away.

- The GDP API and interface is going to change sometime very soon. Some code
  modification will be necessary as and when such change happens.

- The two quick GDP tutorials ([Part 1](https://gdp.cs.berkeley.edu/redmine/projects/gdp/repository/revisions/master/entry/doc/tutorial/gdp-tutorial-part1.md),
[Part 2](https://gdp.cs.berkeley.edu/redmine/projects/gdp/repository/revisions/master/entry/doc/tutorial/gdp-tutorial-part2.md))
are highly recommended.


