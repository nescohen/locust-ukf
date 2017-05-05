#!/bin/bash
USER=nes
HOST=locustmk1.local
ssh -l ${USER} ${HOST} 'bash' < "scripts/remote_build.sh"
