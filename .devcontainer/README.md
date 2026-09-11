# CSC376 dev container

A ready-made Linux environment with Python 3.12, the Robotics Toolbox (installed from
this repo), the Swift simulator and Jupyter. It runs the course notebooks the same way
on Windows, macOS and Linux.

The container uses the host's network (`--net=host`) so the browser tab Swift opens can
connect back to the simulator running inside the container.

## One-time setup (Windows)

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) **4.34 or
   newer**, using the WSL 2 backend (the installer default).
2. Turn on host networking in Docker Desktop:
   1. Sign in to your Docker account in Docker Desktop (this is required for the option).
   2. **Settings → Resources → Network → Enable host networking**.
   3. Select **Apply and restart**.

   If *Enhanced Container Isolation* is turned on, turn it off; host networking does not
   work with it.
3. Install [VS Code](https://code.visualstudio.com/) and the
   [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
   extension.

On macOS the steps are the same. On Linux, Docker Engine supports `--net=host` directly,
so you can skip step 2.

## Opening the course repo

1. Clone this repository and open the folder in VS Code.
2. When prompted, choose **Reopen in Container**. You can also run
   *Dev Containers: Reopen in Container* from the command palette (`Ctrl+Shift+P`).
3. The first build downloads and compiles everything and takes several minutes. Later
   opens are fast.
4. Open a notebook, such as `csc376_practical1.ipynb`. Choose **Select Kernel → Python
   Environments**, then pick the environment at `/opt/venv/bin/python`.

When a notebook calls `env.launch()`, Swift opens a new browser tab on your computer at
`http://localhost:52000`.

## Troubleshooting

**`Could not connect to the Swift simulator`**: the browser tab has 10 seconds to connect.
- Check that host networking is enabled in Docker Desktop (setup step 2). Then run
  *Dev Containers: Rebuild Container*.
- If no tab opened, restart the kernel and try `env.launch(realtime=True, browser="notebook")`
  to show the simulator inside the notebook instead.
- Close old Swift tabs. If one still shows an old page, hard-refresh it (`Ctrl+Shift+R`).

**Blank Swift page, missing robot links, or `ERR_CONTENT_LENGTH_MISMATCH` in the browser
console**: check the Swift tab's address bar. It should show `127.0.0.2:52000`. The
container opens Swift there on purpose, because VS Code forwards `localhost` URLs and its
forwarding cuts the large mesh files short. If the tab shows `localhost` with a different
port, run *Dev Containers: Rebuild Container*, then restart the kernel and close the old
Swift tab. You can also clear old forwards under **Ports → Stop Forwarding Port**.

**Using JupyterLab instead of VS Code notebooks**: in a VS Code terminal, run
`jupyter lab --no-browser`. Then open the `http://localhost:8888/...` link it prints.
Launch Swift with `browser="notebook"`.

**Toolbox changes**: the toolbox is installed in editable mode, so a `git pull` of
Python changes takes effect after a kernel restart. If C++ code changed, run
*Dev Containers: Rebuild Container*.
