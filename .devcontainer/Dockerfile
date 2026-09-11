FROM mcr.microsoft.com/devcontainers/python:1-3.12-bookworm

# gcc/g++ come with the base image; cmake and ninja come from pip (requirements.txt),
# which avoids apt-get and the base image's unsigned Yarn apt repository.

# Virtual environment owned by the non-root user, so pip installs (the editable toolbox
# install in postCreateCommand, plus anything students add later) need no sudo.
# Login shells (which VS Code uses to read the environment) reset PATH in
# /etc/profile.d/00-restore-env.sh, so put the venv back in front afterwards.
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
RUN python -m venv /opt/venv && chown -R vscode:vscode /opt/venv \
    && echo 'export PATH=/opt/venv/bin:$PATH' > /etc/profile.d/99-venv.sh
USER vscode

# Pre-install dependencies so they are cached in the image; the toolbox itself is
# installed from this repo by postCreateCommand (see devcontainer.json)
COPY --chown=vscode:vscode requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# Keep VS Code from port-forwarding the Swift page (see browser.sh and sitecustomize.py)
COPY --chmod=755 browser.sh /usr/local/bin/devcontainer-browser
COPY --chown=vscode:vscode sitecustomize.py /opt/venv/lib/python3.12/site-packages/
