# Running ROS Noetic with Docker 101

This tutorial assume you are running Ubuntu 22.0.4 LTS.

The procedure was tested on a Dell XPS.

1. Install Docker

```bash
chmod +x ./download-docker-desktop.sh
./download-docker-desktop.sh
```
Now, with the deb file downloaded, you can run `./setup.sh`

```bash
./setup.sh
```

2. Allow access to the `/tmp/.X11-unix` directory

  ![alt text](share-directory.png)

3. Run `./build.sh` to build the docker image `my_ros`

4. Run `./run.sh` to create the container, run it and connect your terminal to it. (It also gives you display support, to run GUI programs)

5. If need more terminals, run `./connect.sh` to connect to the existing container.

6. Profit!
