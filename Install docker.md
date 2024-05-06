# Installing Docker

This tutorial is tested on Ubuntu 22.0.4 LTS in WSL 2.

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
