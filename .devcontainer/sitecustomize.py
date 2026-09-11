# Installed into the dev container's venv, so Python runs it at the start of every
# process, including Jupyter kernels.
#
# VS Code sets $BROWSER to its own helper for everything it launches (a devcontainer.json
# remoteEnv cannot override it), and that helper port-forwards localhost URLs. The
# forwarding truncates the Swift simulator's large mesh downloads, so send $BROWSER
# through devcontainer-browser instead (see browser.sh).
import os

_browser = os.environ.get("BROWSER", "")
if _browser.endswith("/helpers/browser.sh"):
    os.environ["DEVCONTAINER_VSCODE_BROWSER"] = _browser
    os.environ["BROWSER"] = "/usr/local/bin/devcontainer-browser"
