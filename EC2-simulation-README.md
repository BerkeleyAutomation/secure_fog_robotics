# EC2 simulation

You can use AWS EC2 for simulation, if you run have limited hardware. Amazon
provides reasonably cheap spot instances with GPUs for graphics applications.
At the time of this writing, g2.2xlarge is around $0.21/hour, and g2.8xlarge
runs for about $0.84/hour; the former should be sufficient for most purposes.

For quickstart, you can use the following AMI as a starting point, which has
most of the software setup already (NVIDIA drivers, X configuration, ROS, HSR
simulator, etc). 

- region: us-west-1
- ami-37d3c857
- instance types: g2.2xlarge; g2.8xlarge

For the moment, this is a private image. If you want access, send your AWS
account number to Nitesh.

Once again, make sure to check the spot prices and budget appropriately.

## How to access the GUI?

Once your instance is up and running, you can use remote desktop over an SSH
tunnel. Here's one way to do it using VNC:

- Log in using SSH with port-fowarding

    ssh -L 127.0.0.1:5900:localhost:5900 ubuntu@<public-ip>

- Set password for user `ubuntu` (use a strong password), you will need it to
  log-in to the GUI.

    sudo passwd ubuntu

- Start the server side of VNC. The following starts up a server on port 5900,
  which is limited to only local connections. However, we should be able to
  access it via port forwarding we setup during SSH earlier.

    sudo x11vnc -localhost -forever -display :0 -auth guess

- With the SSH session running, point your favorite vnc client to
  `localhost:5900`. You should be able to see the login screen.

## Caveats

- Need a reasonably good network connectivity.
- Display resolution is set to somewhat low settings. This is to make sure that
  you can connect using your laptop and not have to deal with scaling issues.
  Changing resolution via 'Display Settings' may or may not work.
