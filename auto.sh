#!/bin/bash
USER=nes
HOST=locustmk1.local
git add -A
git commit
git push
ssh -l ${USER} ${HOST} 'bash' < "scripts/remote_build.sh"
