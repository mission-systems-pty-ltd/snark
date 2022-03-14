FROM ubuntu:20.04
ENV DEBIAN_FRONTEND="noninteractive"
RUN apt update \
    && apt install --yes apt-utils \
    && apt install --yes software-properties-common \
    && apt install --yes ansible git build-essential cmake cmake-curses-gui python3-pip devscripts
RUN apt install --yes python3-stdeb
RUN apt install --yes python3-all
RUN apt install --yes dh-python

RUN apt-get update \
        && apt-get install -y --no-install-recommends git ansible \
        && apt-get clean \
        && rm -rf /var/lib/apt/lists/*

# not necessary, since snark install.yml also installs comma
# just an example in case you need only comma and do not need snark
#RUN mkdir /root/src -p \
#        && cd /root/src \
#        && git clone https://gitlab.com/orthographic/comma.git \
#        && ansible-playbook /root/src/comma/system/ansible/install.python3.yml

RUN mkdir /root/src -p \
        && cd /root/src \
        && git clone https://gitlab.com/orthographic/snark.git \
        && /bin/bash -c "ansible-playbook /root/src/snark/system/ansible/install.minimum.python3.yml"
