#!/bin/sh
# $BROWSER for Python processes in the dev container (installed as
# /usr/local/bin/devcontainer-browser and enabled by sitecustomize.py).
#
# VS Code port-forwards every localhost URL it is asked to open, and that forwarding
# truncates the Swift simulator's large mesh downloads (ERR_CONTENT_LENGTH_MISMATCH).
# The container shares the host network, so Swift's page is reachable directly: swap
# localhost for 127.0.0.2, a loopback address VS Code does not forward, then open the
# URL with VS Code's own helper. Only Swift's http://localhost:<port>/?<port> URLs are
# rewritten; other servers (e.g. JupyterLab) may listen on 127.0.0.1 only.
url=$(printf '%s' "$1" | sed -E 's#^http://(localhost|127\.0\.0\.1):([0-9]+)/\?([0-9]+)$#http://127.0.0.2:\2/?\3#')

if [ -n "$DEVCONTAINER_VSCODE_BROWSER" ]; then
    exec "$DEVCONTAINER_VSCODE_BROWSER" "$url"
fi
echo "Open this URL in your browser: $url"
