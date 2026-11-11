#!/bin/sh
# $BROWSER for Python processes in the dev container (installed as
# /usr/local/bin/devcontainer-browser and enabled by sitecustomize.py).
#
# VS Code port-forwards every localhost URL it is asked to open, and that forwarding
# truncates the Swift simulator's large mesh downloads (ERR_CONTENT_LENGTH_MISMATCH).
# The container shares the host network, so Swift's page is reachable directly.
# VS Code forwards URLs whose host is localhost or 127.0.0.1 (the short form 127.1
# included), but not "localhost." with a trailing dot, which browsers still resolve to
# the loopback address. (Another loopback address such as 127.0.0.2 is not forwarded
# either, but is not reachable on every Windows machine.)
# Only Swift's http://localhost:<port>/?<port> URLs are rewritten.
url=$(printf '%s' "$1" | sed -E 's#^http://(localhost|127\.0\.0\.1):([0-9]+)/\?([0-9]+)$#http://localhost.:\2/?\3#')

if [ -n "$DEVCONTAINER_VSCODE_BROWSER" ]; then
    exec "$DEVCONTAINER_VSCODE_BROWSER" "$url"
fi
echo "Open this URL in your browser: $url"
