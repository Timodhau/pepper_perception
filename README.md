# pepper_perception

Sure! Here’s a Markdown tutorial for the Docker setup and usage you described.

```markdown
# Docker Setup and Usage Tutorial

This tutorial will guide you through the process of building a Docker container, entering it, running a specific script, and stopping the container.

## Prerequisites

Make sure you have Docker installed on your system. If you haven't installed Docker yet, refer to the [Docker installation guide](https://docs.docker.com/get-docker/) for your operating system.

## Steps

### Step 1: Build the Docker Container

To build the Docker container, navigate to the directory containing the `build_docker.sh` script and run:

```bash
./build_docker.sh
```

This script will create the Docker image necessary for your application.

### Step 2: Enter the Docker Container

Once the Docker image is built, you can enter the container using the following command:

```bash
./enter_docker.sh
```

This will start an interactive session within the Docker container.

### Step 3: Run the `start_perceptions.sh` Script

Inside the Docker container, you need to run the `start_perceptions.sh` script. This script requires two parameters: the IP address of PEPPER and the network interface you want to use.

- **PEPPER_IP**: The IP address of your PEPPER robot.
- **NETWORK_INTERFACE**: Use `wlo` for WLAN or `eth` for Ethernet. You can check your network interfaces using the command:

```bash
ifconfig
```

Run the script as follows:

```bash
cd /home/initialization
./start_perceptions.sh PEPPER_IP NETWORK_INTERFACE
```

Replace `PEPPER_IP` with the actual IP address of the PEPPER robot and `NETWORK_INTERFACE` with either `wlo` or `eth` as appropriate.

### Step 4: Stop the Docker Container

To stop the running Docker container, simply execute:

```bash
./close_container.sh
```

```

You can copy and paste this Markdown content into a Markdown editor or viewer to see the formatted tutorial. Let me know if you need any changes or additional information!
